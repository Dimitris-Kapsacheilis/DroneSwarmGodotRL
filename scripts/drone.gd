class_name Drone
extends RigidBody3D


@onready var ai_controller = $AIController3D
@onready var bumper: Area3D = get_node_or_null("Bumper") # Reference to the new Area3D bumper node

@export var drone_color: Color = Color.WHITE
@export var drone_id: int = -1
signal collided(collider: Node)
var target_waypoint: Vector3 = Vector3.INF
var assigned_waypoint_index: int = -1
var formation_offset: Vector3 = Vector3.ZERO
var leader: Drone = null
var is_leader: bool = false
var in_swarm_mode: bool = true
var all_drones: Array[Drone] = []
var current_formation: String = "line"

var swarm_controller: Node = null

var collective := 0.0
var pitch := 0.0
var roll := 0.0
var yaw := 0.0

const FOLLOW_STRENGTH := 14.0
const SMOOTH := 6.0

@onready var zone_manager = get_node_or_null("/root/Swarm Test/NoFlyZoneManager")
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

var coverage := 0.0


var current_target_pos: Vector3 = Vector3.ZERO
var has_target: bool = false


func _ready() -> void:
	if ai_controller!=null:
		ai_controller.init(self)
	_apply_color()
	randomize()
	
	# Connect the Area3D bumper's body_entered signal instead of the rigid body's contact monitor
	if bumper != null:
		if not bumper.body_entered.is_connected(_on_bumper_body_entered):
			bumper.body_entered.connect(_on_bumper_body_entered)
	else:
		push_error("Drone %d: Bumper (Area3D) child node was not found!" % drone_id)
	
	# Place the drone on Layer 1
	set_collision_layer_value(1, true)
	
	# Keep these enabled so the drone physically bounces off Layer 1 and Layer 2
	set_collision_mask_value(1, true)
	set_collision_mask_value(2, true)

func _on_bumper_body_entered(body: Node) -> void:
	# Triggered when the Area3D bumper overlaps with an obstacle on Layer 2
	collided.emit(body)
	
func game_over():
	ai_controller.done = true
	ai_controller.needs_reset = true

func _process(_delta: float) -> void:
	is_in_no_fly_zone()
	
func is_in_no_fly_zone() -> bool:
	if zone_manager == null:
		return false
	for zone in zone_manager.zones:
		zone.update_drone_state(global_position)
		if zone.contains_position(global_position):
			return true
	return false	
	
func _apply_color() -> void:
	var meshes = find_children("*", "MeshInstance3D", true, true)
	if meshes.is_empty():
		push_error("Drone %d: No MeshInstance3D found in your drone.tscn!" % drone_id)
		return

	for mesh: MeshInstance3D in meshes:
		var mat := StandardMaterial3D.new()
		mat.albedo_color = drone_color
		mat.emission_enabled = true
		mat.emission = drone_color * 0.8
		mat.emission_energy_multiplier = 1.5
		mat.roughness = 0.2
		mat.metallic = 0.1
		mesh.material_override = mat

	print("Drone ", drone_id + 1, " colored ", drone_color)

func go_to_waypoint(pos: Vector3, waypoint_index: int = -1) -> void:
	target_waypoint = pos
	assigned_waypoint_index = waypoint_index
	leader = null
	formation_offset = Vector3.ZERO
	in_swarm_mode = false

func set_formation_target(offset: Vector3, new_leader: Drone, formation: String) -> void:
	formation_offset = offset
	leader = new_leader
	current_formation = formation
	target_waypoint = Vector3.INF
	assigned_waypoint_index = -1
	in_swarm_mode = true

func clear_targets() -> void:
	target_waypoint = Vector3.INF
	assigned_waypoint_index = -1
	leader = null

func reset_flight_state(pos: Vector3, rot: Vector3 = Vector3.ZERO) -> void:
	global_position = pos
	global_rotation = rot
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	collective = 0.0
	pitch = 0.0
	roll = 0.0
	yaw = 0.0
	sleeping = false
	clear_targets()
	is_leader = false

func get_rl_observation() -> Dictionary:
	return {
		"id": drone_id,
		"position": _vector3_to_array(global_position),
		"rotation": _vector3_to_array(global_rotation),
		"linear_velocity": _vector3_to_array(linear_velocity),
		"angular_velocity": _vector3_to_array(angular_velocity),
		"is_leader": is_leader,
		"has_waypoint": target_waypoint != Vector3.INF,
		"waypoint": [] if target_waypoint == Vector3.INF else _vector3_to_array(target_waypoint)
	}

func set_boids_data(drones_list: Array[Drone]) -> void:
	all_drones = drones_list

func _physics_process(delta: float) -> void:
	if ai_controller !=null:
		if ai_controller.needs_reset:
			ai_controller.reset()
			#drone.reset()
			return
	
	if target_waypoint != Vector3.INF:
		_go_to_waypoint(delta)
	elif leader != null and in_swarm_mode:
		if current_formation == "boids":
			_boids_behavior(delta)
		else:
			_follow_leader_formation(delta)
	elif is_leader:
		_keyboard_control(delta)
	else:
		linear_damp = 0.8
		angular_damp = 3.0

func _go_to_waypoint(delta: float) -> void:
	var to_target = target_waypoint - global_position
	var distance = to_target.length()
	if distance < 0.5:
		linear_velocity = linear_velocity.lerp(Vector3.ZERO, 15.0 * delta)
		if distance < 0.3:
			if assigned_waypoint_index != -1 and swarm_controller:
				swarm_controller.clear_waypoint_color(assigned_waypoint_index)
			target_waypoint = Vector3.INF
			assigned_waypoint_index = -1
			print("Drone ", drone_id + 1, " arrived at waypoint!")
		return

	var desired_vel = to_target.normalized() * 18.0
	var steering = (desired_vel - linear_velocity) * 28.0
	apply_central_force(steering + Vector3.UP * 9.8 * mass)
	angular_damp = 4.0
	linear_damp = 0.25

func _follow_leader_formation(delta: float) -> void:
	var desired_pos = leader.global_position + leader.global_transform.basis * formation_offset
	var to_target = desired_pos - global_position
	var distance = to_target.length()
	var follow_force = to_target.normalized() * FOLLOW_STRENGTH * min(distance, 20.0)
	var velocity_match = (leader.linear_velocity - linear_velocity) * 5.0
	apply_central_force(follow_force + velocity_match + Vector3.UP * 9.8 * mass * 0.97)
	angular_damp = 3.0
	linear_damp = 0.3

func _boids_behavior(delta: float) -> void:
	var cohesion = Vector3.ZERO
	var separation = Vector3.ZERO
	var velocity_match = Vector3.ZERO
	var count = 0
	for other in all_drones:
		if other == self: continue
		var to_other = other.global_position - global_position
		var dist = to_other.length()
		if dist < 0.01: continue
		cohesion += other.global_position
		count += 1
		if dist < 4.0:
			separation -= to_other.normalized() * (5.0 / dist)
		velocity_match += other.linear_velocity
	if count > 0:
		cohesion = (cohesion / count) - global_position
		velocity_match /= count
	var boids_force = cohesion.normalized() * 11.0 + separation * 18.0 + velocity_match * 4.0
	apply_central_force(boids_force + Vector3.UP * 9.8 * mass * 0.97)
	angular_damp = 4.0
	linear_damp = 0.4

func _keyboard_control(delta: float) -> void:
	var input_thrust = Vector3.ZERO
	var input_yaw = 0.0
	if Input.is_action_pressed("thrust_up"):   input_thrust.y += 1.0
	if Input.is_action_pressed("thrust_down"): input_thrust.y -= 1.0
	if Input.is_action_pressed("pitch_up"):    input_thrust.z += 1.0
	if Input.is_action_pressed("pitch_down"):  input_thrust.z -= 1.0
	if Input.is_action_pressed("roll_left"):   input_thrust.x += 1.0
	if Input.is_action_pressed("roll_right"):  input_thrust.x -= 1.0
	if Input.is_action_pressed("yaw_left"):    input_yaw += 1.0
	if Input.is_action_pressed("yaw_right"):   input_yaw -= 1.0

	collective = lerp(collective, input_thrust.y, SMOOTH * delta)
	pitch     = lerp(pitch,     input_thrust.z, SMOOTH * delta)
	roll      = lerp(roll,      input_thrust.x, SMOOTH * delta)
	yaw       = lerp(yaw,       input_yaw,      SMOOTH * delta)

	var up_force = Vector3.UP * 9.8 * mass + Vector3.UP * collective * 8.0
	var local_force = Vector3(roll * 5.0, 0.0, pitch * 5.0)
	var global_force = global_transform.basis * local_force
	apply_central_force(up_force + global_force)
	apply_torque(Vector3.UP * yaw * 1.5)
	angular_damp = 2.0
	linear_damp = 0.1

func _vector3_to_array(value: Vector3) -> Array[float]:
	return [value.x, value.y, value.z]
