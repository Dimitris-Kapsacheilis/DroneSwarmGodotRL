# camera_follow.gd
extends Camera3D

enum CameraMode {
	FOLLOW,   # Smoothly follows behind the leader / first drone
	MANUAL,   # Free-fly mode using WASD + mouse
	STATIC,   # Top-down orthographic view of the grid
	SIDE      # Side orthographic view of the grid
}

@export var current_mode: CameraMode = CameraMode.STATIC
@export var swarm_controller: Node3D
@export var follow_distance: float = 25.0
@export var height_offset: float = 12.0
@export var smoothness: float = 5.0

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

var yaw: float = 0.0
var pitch: float = 0.0
var is_active_for_input: bool = false
var _manual_initialized: bool = false

func _ready() -> void:
	far = 1000.0
	fov = 65.0
	_init_camera_mode()
	# Defer manual camera positioning so GridManager and Drones are fully loaded
	callable_init_manual.call_deferred()

var callable_init_manual = func():
	if current_mode == CameraMode.MANUAL:
		reset_manual_camera_view()

func _init_camera_mode() -> void:
	match current_mode:
		CameraMode.STATIC:
			projection = Camera3D.PROJECTION_ORTHOGONAL
		CameraMode.SIDE:
			projection = Camera3D.PROJECTION_ORTHOGONAL
		_:
			projection = Camera3D.PROJECTION_PERSPECTIVE

func _input(event: InputEvent) -> void:
	if current_mode != CameraMode.MANUAL:
		return
		
	# Right-click drag to look around in manual mode
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_RIGHT:
		if event.pressed:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			is_active_for_input = true
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			is_active_for_input = false

	if is_active_for_input and event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.005
		pitch -= event.relative.y * 0.005
		pitch = clamp(pitch, -1.57, 1.57)
		rotation = Vector3(pitch, yaw, 0.0)

	# Press 'F' to refocus onto the drones / grid center
	if event is InputEventKey and event.pressed and event.keycode == KEY_F:
		reset_manual_camera_view()

func _process(delta: float) -> void:
	match current_mode:
		CameraMode.MANUAL:
			if not _manual_initialized and is_instance_valid(grid_manager):
				reset_manual_camera_view()
			_manual_camera_movement(delta)
		CameraMode.FOLLOW:
			_follow_leader_camera(delta)
		CameraMode.STATIC:
			_static_camera_tracking(delta)
		CameraMode.SIDE:
			_side_camera_tracking(delta)

# =========================================================================
# MANUAL CAMERA CONTROLS & SPAWN POSITIONING
# =========================================================================

## Places the camera at an elevated 3D diagonal looking directly at the grid/swarm
func reset_manual_camera_view() -> void:
	projection = Camera3D.PROJECTION_PERSPECTIVE
	var target_center = Vector3(50, 10, 50)
	var max_dim = 60.0

	if is_instance_valid(grid_manager):
		target_center = Vector3(
			grid_manager.grid_size.x * 0.5,
			grid_manager.grid_size.y * 0.3,
			grid_manager.grid_size.z * 0.5
		)
		max_dim = max(grid_manager.grid_size.x, grid_manager.grid_size.z)

	var leader = _get_target_drone()
	if is_instance_valid(leader):
		target_center = leader.global_position

	# Position camera diagonally backward and upward
	var cam_offset = Vector3(-max_dim * 0.8, max_dim * 0.7, -max_dim * 0.8)
	global_position = target_center + cam_offset
	look_at(target_center, Vector3.UP)

	# Synchronize rotation variables
	pitch = rotation.x
	yaw = rotation.y
	_manual_initialized = true

func _manual_camera_movement(delta: float) -> void:
	# Only move if right click is holding or if key is pressed
	var move_speed = 40.0 * delta
	var forward = -global_transform.basis.z
	var right = global_transform.basis.x
	var up = Vector3.UP

	var move_vec = Vector3.ZERO
	if Input.is_action_pressed("pitch_up") or Input.is_key_pressed(KEY_W):
		move_vec += forward
	if Input.is_action_pressed("pitch_down") or Input.is_key_pressed(KEY_S):
		move_vec -= forward
	if Input.is_action_pressed("roll_left") or Input.is_key_pressed(KEY_A):
		move_vec -= right
	if Input.is_action_pressed("roll_right") or Input.is_key_pressed(KEY_D):
		move_vec += right
	if Input.is_action_pressed("thrust_up") or Input.is_key_pressed(KEY_E):
		move_vec += up
	if Input.is_action_pressed("thrust_down") or Input.is_key_pressed(KEY_Q):
		move_vec -= up

	if Input.is_key_pressed(KEY_SHIFT):
		move_speed *= 2.5

	global_position += move_vec * move_speed

# =========================================================================
# FOLLOW & ORTHOGRAPHIC MODES
# =========================================================================

func _get_target_drone() -> Node3D:
	if is_instance_valid(swarm_controller):
		if "current_leader" in swarm_controller and is_instance_valid(swarm_controller.current_leader):
			return swarm_controller.current_leader
		if "drones" in swarm_controller and swarm_controller.drones.size() > 0:
			var d = swarm_controller.drones[0]
			if is_instance_valid(d):
				return d

	var drones_in_group = get_tree().get_nodes_in_group("drones")
	if drones_in_group.size() > 0 and is_instance_valid(drones_in_group[0]):
		return drones_in_group[0]

	return null

func _follow_leader_camera(delta: float) -> void:
	projection = Camera3D.PROJECTION_PERSPECTIVE
	var leader = _get_target_drone()
	if not is_instance_valid(leader):
		return

	var behind = -leader.global_transform.basis.z * follow_distance
	var desired_pos = leader.global_position + behind + Vector3.UP * height_offset
	global_position = global_position.lerp(desired_pos, smoothness * delta)
	look_at(leader.global_position + Vector3.UP * 2.0)

func _static_camera_tracking(_delta: float) -> void:
	if not is_instance_valid(grid_manager):
		return
		
	projection = Camera3D.PROJECTION_ORTHOGONAL
	
	var vp = get_viewport()
	var vp_size = vp.get_visible_rect().size if vp else Vector2(1920, 1080)
	var aspect: float = max(0.1, vp_size.x / max(1.0, vp_size.y))

	var grid_w: float = grid_manager.grid_size.x
	var grid_d: float = grid_manager.grid_size.z
	
	var needed_for_depth = grid_d
	var needed_for_width = grid_w / aspect
	size = max(needed_for_depth, needed_for_width) * 1.15

	var grid_center_x: float = grid_w / 2.0
	var grid_center_z: float = grid_d / 2.0
	var camera_height: float = max(grid_w, grid_d) * 2.0
	
	position = Vector3(grid_center_x, camera_height, grid_center_z)
	var target_position = Vector3(grid_center_x, 0.0, grid_center_z)
	look_at(target_position, Vector3.FORWARD)

func _side_camera_tracking(_delta: float) -> void:
	if not is_instance_valid(grid_manager):
		return
		
	projection = Camera3D.PROJECTION_ORTHOGONAL

	var vp = get_viewport()
	var vp_size = vp.get_visible_rect().size if vp else Vector2(1920, 1080)
	var aspect: float = max(0.1, vp_size.x / max(1.0, vp_size.y))

	var grid_h: float = grid_manager.grid_size.y
	var grid_d: float = grid_manager.grid_size.z
	var grid_w: float = grid_manager.grid_size.x

	var needed_for_height = grid_h
	var needed_for_depth = grid_d / aspect
	size = max(needed_for_height, needed_for_depth) * 1.15

	var grid_center_x: float = grid_w / 2.0
	var grid_center_y: float = grid_h / 2.0
	var grid_center_z: float = grid_d / 2.0

	var camera_distance: float = max(grid_w, grid_d) * 2.0
	position = Vector3(grid_center_x + camera_distance, grid_center_y, grid_center_z)

	var target_position = Vector3(grid_center_x, grid_center_y, grid_center_z)
	look_at(target_position, Vector3.UP)
