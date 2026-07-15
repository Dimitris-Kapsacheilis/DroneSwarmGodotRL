# camera_follow.gd
# Attach this to a Camera3D node (make it "Current")

extends Camera3D

# Define the available camera modes
enum CameraMode {
	FOLLOW,    # Smoothly follows behind the leader drone
	MANUAL,    # Free-fly mode using WASD + mouse
	STATIC     # Remains at a fixed position, tracking the leader with rotation
}
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

@export var swarm_controller: Node3D          # Drag your swarm root Node3D here
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
	
	# Initialize mouse mode based on starting camera mode
	_apply_mouse_mode()


func _input(event: InputEvent) -> void:
	# Only capture mouse inputs if we are in free-fly manual mode
	if current_mode != CameraMode.MANUAL:
		return
	
	if event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.005
		pitch -= event.relative.y * 0.005
		pitch = clamp(pitch, -1.57, 1.57)  # Prevent flipping over
		rotation = Vector3(pitch, yaw, 0.0)


func _process(delta: float) -> void:
	# === TOGGLE CAMERA MODES ===
	if Input.is_action_just_pressed("toggle_camera_mode"):
		# Cycle through modes: FOLLOW (0) -> MANUAL (1) -> STATIC (2) -> FOLLOW (0)
		var next_mode_index = (int(current_mode) + 1) % 3
		current_mode = next_mode_index as CameraMode
		_apply_mouse_mode()
	
	# Execute logic depending on the active state
	match current_mode:
		CameraMode.MANUAL:
			_manual_camera_movement(delta)
		CameraMode.FOLLOW:
			_follow_leader_camera(delta)
		CameraMode.STATIC:
			_static_camera_tracking(delta)


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
			print("Camera: STATIC MODE (locked position, watching leader)")


func _manual_camera_movement(delta: float) -> void:
	var move_speed = 35.0 * delta
	
	var forward = -global_transform.basis.z
	var right   = global_transform.basis.x
	var up      = global_transform.basis.y
	
	if Input.is_action_pressed("pitch_up"):      global_position += forward * move_speed   # W
	if Input.is_action_pressed("pitch_down"):    global_position -= forward * move_speed   # S
	if Input.is_action_pressed("roll_left"):     global_position -= right   * move_speed   # A
	if Input.is_action_pressed("roll_right"):    global_position += right   * move_speed   # D
	if Input.is_action_pressed("thrust_up"):     global_position += up      * move_speed   # Space
	if Input.is_action_pressed("thrust_down"):   global_position -= up      * move_speed   # Ctrl
	
	# Hold Shift for faster movement
	if Input.is_key_pressed(KEY_SHIFT):
		global_position += (forward * move_speed * 2.0) if Input.is_action_pressed("pitch_up") else Vector3.ZERO


func _follow_leader_camera(delta: float) -> void:
	if swarm_controller == null or not is_instance_valid(swarm_controller.current_leader):
		return
		
	var leader = swarm_controller.current_leader
	var behind = -leader.global_transform.basis.z * follow_distance
	var desired_pos = leader.global_position + behind + Vector3.UP * height_offset
	
	global_position = global_position.lerp(desired_pos, smoothness * delta)
	look_at(leader.global_position + Vector3.UP * 2.0)


func _static_camera_tracking(_delta: float) -> void:
	projection = Camera3D.PROJECTION_ORTHOGONAL
	size = grid_manager.grid_size.y*1.05 # Adjust this value to zoom in or out of the 30x30 area
	# Horizontal center of the 30x30 grid (0 to 30 range, so center is 15)
	var grid_center_x: float = grid_manager.grid_size.x/2
	var grid_center_z: float = grid_manager.grid_size.z/2
	
	# Position the camera well above the top boundary of the grid (Y = 30)
	var camera_height: float = grid_manager.grid_size.y*1.8
	
	position = Vector3(grid_center_x, camera_height, grid_center_z)
	
	# Target the center point on the ground level of the grid
	var target_position = Vector3(grid_center_x, 0.0, grid_center_z)
	
	# Since looking straight down is parallel to the Y-axis, 
	# we use Vector3.FORWARD as the temporary 'UP' vector to prevent camera flipping.
	look_at(target_position, Vector3.FORWARD)
