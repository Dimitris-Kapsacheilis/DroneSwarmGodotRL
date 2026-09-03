# camera_follow.gd
# Attach this to a Camera3D node (make it "Current")
extends Camera3D

# Define the available camera modes
enum CameraMode {
	FOLLOW,   # Smoothly follows behind the leader / first drone
	MANUAL,   # Free-fly mode using WASD + mouse
	STATIC,   # Top-down orthographic view of the grid
	SIDE      # Side orthographic view of the grid (for better height/depth visualization)
}

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@export var swarm_controller: Node3D  # Drag your swarm root Node3D here
@export var follow_distance: float = 25.0
@export var height_offset: float = 12.0
@export var smoothness: float = 5.0

# Exported so you can set the default starting mode in the Inspector
@export var current_mode: CameraMode = CameraMode.STATIC

# Camera rotation tracking for manual mode
var yaw: float = 0.0
var pitch: float = 0.0

func _ready() -> void:
	current = true
	far = 300.0
	fov = 65.0
	_apply_mouse_mode()

func _input(event: InputEvent) -> void:
	if current_mode != CameraMode.MANUAL:
		return
	if event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.005
		pitch -= event.relative.y * 0.005
		pitch = clamp(pitch, -1.57, 1.57)
		rotation = Vector3(pitch, yaw, 0.0)

func _process(delta: float) -> void:
	if Input.is_action_just_pressed("toggle_camera_mode"):
		var next_mode_index = (int(current_mode) + 1) % 4
		current_mode = next_mode_index as CameraMode
		_apply_mouse_mode()

	match current_mode:
		CameraMode.MANUAL:
			_manual_camera_movement(delta)
		CameraMode.FOLLOW:
			_follow_leader_camera(delta)
		CameraMode.STATIC:
			_static_camera_tracking(delta)
		CameraMode.SIDE:
			_side_camera_tracking(delta)

func _apply_mouse_mode() -> void:
	match current_mode:
		CameraMode.MANUAL:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			yaw = rotation.y
			pitch = rotation.x
			print("Camera: MANUAL FREE-FLY MODE (WASD + mouse look)")
		CameraMode.FOLLOW:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			print("Camera: FOLLOW MODE (auto tracks leader)")
		CameraMode.STATIC:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			print("Camera: STATIC MODE (top-down, locked position)")
		CameraMode.SIDE:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			print("Camera: SIDE MODE (side orthographic view of the grid)")

func _manual_camera_movement(delta: float) -> void:
	projection = Camera3D.PROJECTION_PERSPECTIVE
	var move_speed = 35.0 * delta
	var forward = -global_transform.basis.z
	var right = global_transform.basis.x
	var up = global_transform.basis.y
	if Input.is_action_pressed("pitch_up"): global_position += forward * move_speed
	if Input.is_action_pressed("pitch_down"): global_position -= forward * move_speed
	if Input.is_action_pressed("roll_left"): global_position -= right * move_speed
	if Input.is_action_pressed("roll_right"): global_position += right * move_speed
	if Input.is_action_pressed("thrust_up"): global_position += up * move_speed
	if Input.is_action_pressed("thrust_down"): global_position -= up * move_speed
	if Input.is_key_pressed(KEY_SHIFT):
		global_position += (forward * move_speed * 2.0) if Input.is_action_pressed("pitch_up") else Vector3.ZERO

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
	size = grid_manager.grid_size.y * 1.05
	var grid_center_x: float = grid_manager.grid_size.x / 2.0
	var grid_center_z: float = grid_manager.grid_size.z / 2.0
	var camera_height: float = grid_manager.grid_size.y * 1.8
	position = Vector3(grid_center_x, camera_height, grid_center_z)
	var target_position = Vector3(grid_center_x, 0.0, grid_center_z)
	look_at(target_position, Vector3.FORWARD)

func _side_camera_tracking(_delta: float) -> void:
	if not is_instance_valid(grid_manager):
		return
	projection = Camera3D.PROJECTION_ORTHOGONAL
	size = max(grid_manager.grid_size.y, grid_manager.grid_size.z) * 1.05

	var grid_center_x: float = grid_manager.grid_size.x / 2.0
	var grid_center_y: float = grid_manager.grid_size.y / 2.0
	var grid_center_z: float = grid_manager.grid_size.z / 2.0

	var camera_distance: float = grid_manager.grid_size.x * 1.8
	position = Vector3(grid_center_x + camera_distance, grid_center_y, grid_center_z)

	var target_position = Vector3(grid_center_x, grid_center_y, grid_center_z)
	look_at(target_position, Vector3.UP)
