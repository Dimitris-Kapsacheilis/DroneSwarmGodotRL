# camera_follow.gd
# Attach this to a Camera3D node (make it "Current")

extends Camera3D

@export var swarm_controller: Node3D          # Drag your swarm root Node3D here
@export var follow_distance: float = 25.0
@export var height_offset: float = 12.0
@export var smoothness: float = 5.0

# Camera mode
var manual_mode: bool = false
var yaw: float = 0.0
var pitch: float = 0.0

func _ready() -> void:
	current = true
	far = 300.0
	fov = 65.0
	# Uncomment for nice background blur:
	# dof_blur_far_enabled = true
	# dof_blur_far_distance = 80.0
	# dof_blur_far_transition = 30.0


func _input(event: InputEvent) -> void:
	if not manual_mode:
		return
	
	if event is InputEventMouseMotion:
		yaw -= event.relative.x * 0.005
		pitch -= event.relative.y * 0.005
		pitch = clamp(pitch, -1.57, 1.57)  # prevent flipping over
		rotation = Vector3(pitch, yaw, 0.0)


func _process(delta: float) -> void:
	# === TOGGLE MANUAL / FOLLOW MODE (only once per press) ===
	if Input.is_action_just_pressed("toggle_camera_mode"):
		manual_mode = not manual_mode
		
		if manual_mode:
			Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
			yaw = rotation.y
			pitch = rotation.x
			print("Camera: MANUAL FREE-FLY MODE (WASD + mouse look)")
		else:
			Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)
			print("Camera: FOLLOW MODE (auto tracks leader)")
	
	if manual_mode:
		_manual_camera_movement(delta)
	else:
		_follow_leader_camera(delta)


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
	
	# Optional: hold Shift for faster movement
	if Input.is_key_pressed(KEY_SHIFT):
		global_position += (forward * move_speed * 2.0) if Input.is_action_pressed("pitch_up") else Vector3.ZERO


func _follow_leader_camera(delta: float) -> void:
	if not swarm_controller or not swarm_controller.current_leader:
		return
	
	var leader = swarm_controller.current_leader
	
	var behind = -leader.global_transform.basis.z * follow_distance
	var desired_pos = leader.global_position + behind + Vector3.UP * height_offset
	
	global_position = global_position.lerp(desired_pos, smoothness * delta)
	look_at(leader.global_position + Vector3.UP * 2.0)
