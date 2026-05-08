# swarm_controller.gd
extends Node3D

@export var drone_packed_scene: PackedScene
@export var num_drones: int = 6
@export var spawn_height: float = 8.0
@export var follow_distance: float = 7.0
@export var follow_spread: float = 4.5

@export var waypoints: Array[Vector3] = [
	Vector3(0, 12, 0),
	Vector3(20, 18, -30),
	Vector3(-25, 15, 25),
	Vector3(35, 22, 10),
	Vector3(-10, 10, -45)
]

var drones: Array[Drone] = []
var leader_index: int = 0
var current_formation: String = "line"
var current_leader: Drone
var waypoint_mode: bool = false
var individual_mode: bool = false
var selected_drone_index: int = 0

var waypoint_markers: Array[MeshInstance3D] = []
var default_waypoint_color := Color(1, 0, 0, 0.7)

var _drone_colors := [
	Color(1.0, 0.2, 0.2),   # Red
	Color(0.2, 1.0, 0.2),   # Green
	Color(0.2, 0.6, 1.0),   # Blue
	Color(1.0, 1.0, 0.2),   # Yellow
	Color(1.0, 0.5, 0.0),   # Orange
	Color(0.8, 0.2, 1.0)    # Purple
]

func _ready() -> void:
	if not drone_packed_scene:
		push_error("Assign your drone.tscn!")
		return

	for i in num_drones:
		var drone: Drone = drone_packed_scene.instantiate()
		
		# IMPORTANT: Set all properties BEFORE add_child so _ready() sees them
		drone.drone_id = i
		drone.drone_color = _drone_colors[i % _drone_colors.size()]
		drone.swarm_controller = self
		
		add_child(drone)                    # ← _ready() now gets correct color
		drone.global_position = Vector3(
			(i - float(num_drones - 1) / 2.0) * 5.0,
			spawn_height,
			0.0
		)
		drones.append(drone)

	_create_waypoint_markers()
	set_leader(0)
	set_formation("line")

	print("🎉 Swarm ready with UNIQUE COLORS!")
	print("I = toggle Swarm / Individual mode")
	print("TAB = toggle Waypoint Mode")

func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("toggle_waypoint"):
		waypoint_mode = not waypoint_mode
		print("Waypoint Mode: ", "ENABLED" if waypoint_mode else "DISABLED")

	if Input.is_key_pressed(KEY_I):
		individual_mode = not individual_mode
		waypoint_mode = false
		print("Control Mode → ", "INDIVIDUAL" if individual_mode else "SWARM")
		if not individual_mode:
			for d in drones: d.clear_targets()

	if individual_mode:
		_handle_individual_input()
	else:
		_handle_swarm_input()

# (the rest of your input handlers, set_leader, set_formation, _update_formation_targets, _get_formation_offset stay exactly the same as my previous version)

func _handle_swarm_input() -> void:
	for i in range(num_drones):
		if Input.is_key_pressed(KEY_1 + i):
			set_leader(i)
	if Input.is_key_pressed(KEY_7): set_formation("line")
	if Input.is_key_pressed(KEY_8): set_formation("v")
	if Input.is_key_pressed(KEY_9): set_formation("circle")
	if Input.is_key_pressed(KEY_0): set_formation("grid")
	if Input.is_key_pressed(KEY_MINUS): set_formation("diamond")
	if Input.is_key_pressed(KEY_EQUAL): set_formation("boids")

	if waypoint_mode:
		for i in min(5, waypoints.size()):
			if Input.is_key_pressed(KEY_1 + i):
				current_leader.go_to_waypoint(waypoints[i], i)
				_set_waypoint_color(i, current_leader.drone_color)
				print("→ Leader heading to Waypoint #", i + 1)

func _handle_individual_input() -> void:
	for i in range(num_drones):
		if Input.is_key_pressed(KEY_1 + i):
			selected_drone_index = i
			drones[i].is_leader = true
			print("Selected Drone #", i + 1, " (", drones[i].drone_color, ")")
	for i in min(5, waypoints.size()):
		if Input.is_key_pressed(KEY_1 + i):
			drones[selected_drone_index].go_to_waypoint(waypoints[i], i)
			_set_waypoint_color(i, drones[selected_drone_index].drone_color)
			print("Drone #", selected_drone_index + 1, " → Waypoint #", i + 1)

func set_leader(new_index: int) -> void:
	if new_index == leader_index: return
	drones[leader_index].is_leader = false
	leader_index = new_index
	drones[leader_index].is_leader = true
	current_leader = drones[leader_index]
	print("→ Leader is now Drone #", leader_index + 1)

func set_formation(new_formation: String) -> void:
	if new_formation == current_formation: return
	current_formation = new_formation
	print("→ Formation: ", current_formation.to_upper())
	_update_formation_targets()

func _update_formation_targets() -> void:
	var leader = drones[leader_index]
	var num_followers = drones.size() - 1
	for i in drones.size():
		if i == leader_index: continue
		var follower = drones[i]
		var rank = i if i < leader_index else i - 1
		var offset = _get_formation_offset(rank, num_followers)
		follower.set_formation_target(offset, leader, current_formation)

	if current_formation == "boids":
		for d in drones:
			d.set_boids_data(drones)

func _get_formation_offset(rank: int, num_followers: int) -> Vector3:
	match current_formation:
		"line":   return Vector3((rank - (num_followers - 1) / 2.0) * follow_spread, fmod(rank, 3.0) * 0.8 - 0.8, -follow_distance)
		"v":      var side = 1 if rank < num_followers / 2.0 else -1; var row = abs(rank - (num_followers / 2.0)); return Vector3(row * follow_spread * side, 0, -follow_distance + row * 2.0)
		"circle": var angle = rank * (2 * PI / num_followers); var radius = follow_distance * 1.1; return Vector3(sin(angle) * radius, 0, -cos(angle) * radius)
		"grid":   var cols = ceil(sqrt(num_followers)); var row = floor(rank / cols); var col = fmod(rank, cols); return Vector3((col - (cols - 1) / 2.0) * follow_spread, fmod(row, 2.0) * 0.6 - 0.3, -follow_distance - row * follow_spread * 0.8)
		"diamond":var mod_rank = fmod(rank, 4.0); return Vector3((mod_rank - 1.5) * follow_spread * 0.7, 0, -follow_distance + abs(mod_rank - 1.5) * 2.0 - 3.0)
		_:        return Vector3(0, 0, -follow_distance)

# === WAYPOINT MARKERS (now guaranteed to change color) ===
func _create_waypoint_markers() -> void:
	var sphere_mesh = SphereMesh.new()
	sphere_mesh.radius = 0.5
	sphere_mesh.height = 1.0
	for i in waypoints.size():
		var marker = MeshInstance3D.new()
		marker.mesh = sphere_mesh
		var mat = StandardMaterial3D.new()
		mat.albedo_color = default_waypoint_color
		mat.emission_enabled = true
		mat.emission = default_waypoint_color * 0.4
		marker.material_override = mat
		marker.global_position = waypoints[i]
		marker.name = "WaypointMarker_" + str(i + 1)
		add_child(marker)
		waypoint_markers.append(marker)

		var label = Label3D.new()
		label.text = str(i + 1)
		label.font_size = 0.5
		label.pixel_size = 0.05
		label.global_position = waypoints[i] + Vector3(0, 1.0, 0)
		label.outline_size = 0.1
		label.outline_modulate = Color(0,0,0)
		add_child(label)

func _set_waypoint_color(idx: int, color: Color) -> void:
	if idx < 0 or idx >= waypoint_markers.size(): return
	var mat = waypoint_markers[idx].material_override as StandardMaterial3D
	if mat:
		mat.albedo_color = color
		mat.emission = color * 0.8

func clear_waypoint_color(idx: int) -> void:
	if idx < 0 or idx >= waypoint_markers.size(): return
	var mat = waypoint_markers[idx].material_override as StandardMaterial3D
	if mat:
		mat.albedo_color = default_waypoint_color
		mat.emission = default_waypoint_color * 0.4

func _physics_process(delta: float) -> void:
	if not individual_mode and current_formation != "boids":
		_update_formation_targets()
