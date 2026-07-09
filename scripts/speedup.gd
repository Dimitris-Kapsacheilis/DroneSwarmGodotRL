extends Node

@export var VSYNC = false
@export var max_fps = 0
@export var time_scale = 5.0
@export var physics_ticks_per_second = 40
@export var max_physics_steps_per_frame = 2


func _ready() -> void:
	# 1. Optimize rendering overhead
	if VSYNC == false:
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	Engine.max_fps = max_fps
	
	# 2. Speed up the physical time passage
	Engine.time_scale = time_scale # Tweak this to find your system's stability limit
	
	# 3. Reduce physics fidelity if precision limits allow
	Engine.physics_ticks_per_second = physics_ticks_per_second
	
	# 4. Prevent excessive catch-up steps during lag spikes
	Engine.max_physics_steps_per_frame = max_physics_steps_per_frame
