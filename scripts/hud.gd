extends Control

@onready var coveragetext = $Coverage
@onready var camera_mode_text = $CameraMode
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	# Update coverage text
	if grid_manager != null:
		coveragetext.text = "%.2f%%" % grid_manager.get_coverage_percentage()

	# Update camera mode text
	var cam = get_viewport().get_camera_3d()
	if cam != null and "current_mode" in cam:
		var mode_name = cam.CameraMode.keys()[cam.current_mode]
		camera_mode_text.text = "Camera Mode : " + mode_name
