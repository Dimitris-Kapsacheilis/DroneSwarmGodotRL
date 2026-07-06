extends Node3D

const Drone = preload("res://scripts/drone.gd")
var rng = RandomNumberGenerator.new()
@export var drone_packed_scene: PackedScene
@export var num_drones: int = 1
@export var spawn_height: float = 8.0
@export var follow_distance: float = 7.0
@export var follow_spread: float = 4.5
@export var grid_manager: Node3D # Assign your GridManager here in the Inspector

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

const ACTION_TOGGLE_INDIVIDUAL := "toggle_individual"
const DRONE_SELECT_ACTIONS := [
	"select_drone_1",
	"select_drone_2",
	"select_drone_3",
	"select_drone_4",
	"select_drone_5",
	"select_drone_6"
]
const DRONE_SELECT_KEYS := [KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6]
const WAYPOINT_ACTIONS := [
	"waypoint_1",
	"waypoint_2",
	"waypoint_3",
	"waypoint_4",
	"waypoint_5"
]
const WAYPOINT_KEYS := [KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5]
const FORMATION_ACTIONS := {
	"formation_line": "line",
	"formation_v": "v",
	"formation_circle": "circle",
	"formation_grid": "grid",
	"formation_diamond": "diamond",
	"formation_boids": "boids"
}
const FORMATION_KEYS := {
	"formation_line": KEY_7,
	"formation_v": KEY_8,
	"formation_circle": KEY_9,
	"formation_grid": KEY_0,
	"formation_diamond": KEY_MINUS,
	"formation_boids": KEY_EQUAL
}

func _ready() -> void:
	_ensure_input_actions()

	if not drone_packed_scene:
		push_error("Assign your drone.tscn!")
		return

	for i in range(num_drones):
		var drone: Drone = drone_packed_scene.instantiate()

		# Set exported values before add_child so the drone's _ready() sees them.
		drone.drone_id = i
		drone.drone_color = _drone_colors[i % _drone_colors.size()]
		drone.swarm_controller = self

		add_child(drone)
		drone.global_position = Vector3(
			(i - float(num_drones - 1) / 2.0) * 5.0,
			spawn_height,
			0.0
		)
		drones.append(drone)
		if grid_manager:
			grid_manager.drone = drone

	if drones.is_empty():
		push_error("Swarm has no drones. Increase num_drones above zero.")
		return
	# Pass the reference directly to the GridManager
	else :
		reset_swarm_pos()
	_create_waypoint_markers()
	set_leader(0)
	set_formation("line")

	print("Swarm ready with ", drones.size(), " drones.")
	print("I = toggle swarm/individual mode, TAB = toggle waypoint mode")
	print("1-6 = select leader/drone, F1-F5 = assign waypoint")

func reset_swarm_pos() -> void:
	for drone in drones:
		var i = 0
		drone.global_position = Vector3(
			rng.randf_range(0,grid_manager.grid_size.x),
			rng.randf_range(0,grid_manager.grid_size.y),
			rng.randf_range(0,grid_manager.grid_size.z)
			)
		
	
	

func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("toggle_waypoint"):
		waypoint_mode = not waypoint_mode
		print("Waypoint Mode: ", "ENABLED" if waypoint_mode else "DISABLED")

	if Input.is_action_just_pressed(ACTION_TOGGLE_INDIVIDUAL):
		individual_mode = not individual_mode
		waypoint_mode = false
		print("Control Mode -> ", "INDIVIDUAL" if individual_mode else "SWARM")
		if not individual_mode:
			for d in drones:
				d.clear_targets()
			set_leader(clamp(selected_drone_index, 0, drones.size() - 1))
			set_formation(current_formation)
		else:
			_select_individual_drone(clamp(leader_index, 0, drones.size() - 1))

	if individual_mode:
		_handle_individual_input()
	else:
		_handle_swarm_input()

func _ensure_input_actions() -> void:
	_ensure_key_action(ACTION_TOGGLE_INDIVIDUAL, KEY_I)

	for i in range(DRONE_SELECT_ACTIONS.size()):
		_ensure_key_action(DRONE_SELECT_ACTIONS[i], DRONE_SELECT_KEYS[i])

	for i in range(WAYPOINT_ACTIONS.size()):
		_ensure_key_action(WAYPOINT_ACTIONS[i], WAYPOINT_KEYS[i])

	for action in FORMATION_ACTIONS.keys():
		_ensure_key_action(action, FORMATION_KEYS[action])


func _ensure_key_action(action: StringName, physical_keycode: Key) -> void:
	if not InputMap.has_action(action):
		InputMap.add_action(action)

	if not InputMap.action_get_events(action).is_empty():
		return

	var event := InputEventKey.new()
	event.physical_keycode = physical_keycode
	InputMap.action_add_event(action, event)


func _handle_swarm_input() -> void:
	for i in range(min(drones.size(), DRONE_SELECT_ACTIONS.size())):
		if Input.is_action_just_pressed(DRONE_SELECT_ACTIONS[i]):
			set_leader(i)

	for action in FORMATION_ACTIONS.keys():
		if Input.is_action_just_pressed(action):
			set_formation(FORMATION_ACTIONS[action])

	if waypoint_mode:
		for i in range(min(5, waypoints.size())):
			if Input.is_action_just_pressed(WAYPOINT_ACTIONS[i]):
				current_leader.go_to_waypoint(waypoints[i], i)
				_set_waypoint_color(i, current_leader.drone_color)
				print("Leader heading to Waypoint #", i + 1)

func _handle_individual_input() -> void:
	for i in range(min(drones.size(), DRONE_SELECT_ACTIONS.size())):
		if Input.is_action_just_pressed(DRONE_SELECT_ACTIONS[i]):
			_select_individual_drone(i)

	for i in range(min(5, waypoints.size())):
		if Input.is_action_just_pressed(WAYPOINT_ACTIONS[i]):
			drones[selected_drone_index].go_to_waypoint(waypoints[i], i)
			_set_waypoint_color(i, drones[selected_drone_index].drone_color)
			print("Drone #", selected_drone_index + 1, " -> Waypoint #", i + 1)


func _select_individual_drone(index: int) -> void:
	if index < 0 or index >= drones.size():
		return

	selected_drone_index = index
	for drone in drones:
		drone.is_leader = false

	drones[selected_drone_index].is_leader = true
	current_leader = drones[selected_drone_index]
	print("Selected Drone #", selected_drone_index + 1, " (", drones[selected_drone_index].drone_color, ")")

func set_leader(new_index: int) -> void:
	if new_index < 0 or new_index >= drones.size():
		push_warning("Ignoring invalid leader index: %d" % new_index)
		return

	if leader_index >= 0 and leader_index < drones.size():
		drones[leader_index].is_leader = false

	leader_index = new_index
	drones[leader_index].is_leader = true
	current_leader = drones[leader_index]
	selected_drone_index = leader_index
	print("Leader is now Drone #", leader_index + 1)
	_update_formation_targets()

func set_formation(new_formation: String) -> void:
	if new_formation == current_formation and _followers_have_targets():
		return

	current_formation = new_formation
	print("Formation: ", current_formation.to_upper())
	_update_formation_targets()


func _followers_have_targets() -> bool:
	for i in range(drones.size()):
		if i == leader_index:
			continue
		if drones[i].leader == null:
			return false
	return true

func _update_formation_targets() -> void:
	if drones.is_empty() or leader_index < 0 or leader_index >= drones.size():
		return

	var leader = drones[leader_index]
	var num_followers = drones.size() - 1
	for i in range(drones.size()):
		if i == leader_index:
			continue
		var follower = drones[i]
		var rank = i if i < leader_index else i - 1
		var offset = _get_formation_offset(rank, num_followers)
		follower.set_formation_target(offset, leader, current_formation)

	if current_formation == "boids":
		for d in drones:
			d.set_boids_data(drones)

func _get_formation_offset(rank: int, num_followers: int) -> Vector3:
	if num_followers <= 0:
		return Vector3.ZERO

	match current_formation:
		"line":
			return Vector3((rank - (num_followers - 1) / 2.0) * follow_spread, fmod(rank, 3.0) * 0.8 - 0.8, -follow_distance)
		"v":
			var side = 1 if rank < num_followers / 2.0 else -1
			var row = abs(rank - (num_followers / 2.0))
			return Vector3(row * follow_spread * side, 0, -follow_distance + row * 2.0)
		"circle":
			var angle = rank * (2 * PI / num_followers)
			var radius = follow_distance * 1.1
			return Vector3(sin(angle) * radius, 0, -cos(angle) * radius)
		"grid":
			var cols = ceil(sqrt(num_followers))
			var grid_row = floor(rank / cols)
			var col = fmod(rank, cols)
			return Vector3((col - (cols - 1) / 2.0) * follow_spread, fmod(grid_row, 2.0) * 0.6 - 0.3, -follow_distance - grid_row * follow_spread * 0.8)
		"diamond":
			var mod_rank = fmod(rank, 4.0)
			return Vector3((mod_rank - 1.5) * follow_spread * 0.7, 0, -follow_distance + abs(mod_rank - 1.5) * 2.0 - 3.0)
		_:
			return Vector3(0, 0, -follow_distance)

func _create_waypoint_markers() -> void:
	var sphere_mesh = SphereMesh.new()
	sphere_mesh.radius = 0.5
	sphere_mesh.height = 1.0
	for i in range(waypoints.size()):
		var marker = MeshInstance3D.new()
		marker.mesh = sphere_mesh
		var mat = StandardMaterial3D.new()
		mat.albedo_color = default_waypoint_color
		mat.emission_enabled = true
		mat.emission = default_waypoint_color * 0.4
		marker.material_override = mat
		marker.name = "WaypointMarker_" + str(i + 1)
		add_child(marker)
		marker.global_position = waypoints[i]
		waypoint_markers.append(marker)

		var label = Label3D.new()
		label.text = str(i + 1)
		label.font_size = 6.0
		label.pixel_size = 1.0
		label.outline_size = 0.1
		label.outline_modulate = Color(0,0,0)
		add_child(label)
		label.global_position = waypoints[i] + Vector3(0, 1.0, 0)

func _set_waypoint_color(idx: int, color: Color) -> void:
	if idx < 0 or idx >= waypoint_markers.size():
		return
	var mat = waypoint_markers[idx].material_override as StandardMaterial3D
	if mat:
		mat.albedo_color = color
		mat.emission = color * 0.8

func clear_waypoint_color(idx: int) -> void:
	if idx < 0 or idx >= waypoint_markers.size():
		return
	var mat = waypoint_markers[idx].material_override as StandardMaterial3D
	if mat:
		mat.albedo_color = default_waypoint_color
		mat.emission = default_waypoint_color * 0.4

func _physics_process(_delta: float) -> void:
	if not individual_mode and current_formation != "boids":
		_update_formation_targets()
