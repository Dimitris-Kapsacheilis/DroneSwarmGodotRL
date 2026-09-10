extends Control

@onready var coveragetext: Label = get_node_or_null("Coverage")
@onready var camera_mode_text: Label = get_node_or_null("CameraMode")
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

# --- UI Slider Nodes ---
@onready var speed_slider: HSlider = get_node_or_null("DroneSpeed/SpeedSlider")
@onready var speed_label: Label = get_node_or_null("DroneSpeed/SpeedLabel")

# --- Container for dynamic checkboxes ---
@onready var trail_toggles_container: VBoxContainer = get_node_or_null("TrailToggles")

var master_all_checkbox: CheckBox = null
var created_drone_checkboxes: Dictionary = {}
var _is_updating_toggles: bool = false

func _ready() -> void:
	# 1. Setup Speed Slider
	if speed_slider != null:
		speed_slider.value_changed.connect(_on_speed_slider_changed)
		_update_drone_speeds(speed_slider.value)
		if speed_label != null:
			speed_label.text = "Drone Speed: %.0f" % speed_slider.value

	# 2. Setup Checkboxes
	_setup_trail_toggle_ui()

func _process(_delta: float) -> void:
	# Update coverage text
	if grid_manager != null and coveragetext != null:
		coveragetext.text = "%.2f%%" % grid_manager.get_coverage_percentage()

	# Update camera mode text
	var cam = get_viewport().get_camera_3d()
	if cam != null and "current_mode" in cam and camera_mode_text != null:
		var mode_name = cam.CameraMode.keys()[cam.current_mode]
		camera_mode_text.text = "Camera Mode : " + mode_name

	# Dynamically register drone checkboxes as drones spawn
	_sync_drone_checkboxes()

# =====================================================
# SPEED SLIDER LOGIC
# =====================================================

func _on_speed_slider_changed(value: float) -> void:
	if speed_label != null:
		speed_label.text = "Drone Speed: %.0f" % value
	_update_drone_speeds(value)

func _update_drone_speeds(new_speed: float) -> void:
	var drones = get_tree().get_nodes_in_group("drones")
	for drone in drones:
		if "flight_speed" in drone:
			drone.flight_speed = new_speed
		for child in drone.get_children():
			if "flight_speed" in child:
				child.flight_speed = new_speed

# =====================================================
# DYNAMIC TRAIL CHECKBOX UI
# =====================================================

func _setup_trail_toggle_ui() -> void:
	if trail_toggles_container == null:
		return

	for child in trail_toggles_container.get_children():
		child.queue_free()

	# Master Toggle: "All Trails"
	master_all_checkbox = CheckBox.new()
	master_all_checkbox.text = "All Trails"
	master_all_checkbox.button_pressed = true
	master_all_checkbox.toggled.connect(_on_all_trails_toggled)
	trail_toggles_container.add_child(master_all_checkbox)

func _sync_drone_checkboxes() -> void:
	if trail_toggles_container == null or grid_manager == null:
		return

	var drones = get_tree().get_nodes_in_group("drones")
	for drone in drones:
		if not is_instance_valid(drone):
			continue

		var d_id: int = drone.drone_id if "drone_id" in drone else drone.get_instance_id()
		
		if not created_drone_checkboxes.has(d_id):
			var d_color: Color = drone.drone_color if "drone_color" in drone else Color.WHITE
			var cb = CheckBox.new()
			var name_label = "Drone %d Trail" % (drone.drone_id + 1 if drone.drone_id >= 0 else d_id)
			cb.text = name_label
			cb.button_pressed = master_all_checkbox.button_pressed if master_all_checkbox else true
			
			cb.add_theme_color_override("font_color", d_color)
			cb.add_theme_color_override("font_pressed_color", d_color)
			cb.add_theme_color_override("font_hover_color", d_color.lightened(0.2))

			cb.toggled.connect(func(toggled_on: bool):
				_on_single_drone_toggled(d_id, toggled_on)
			)

			trail_toggles_container.add_child(cb)
			created_drone_checkboxes[d_id] = cb

func _on_single_drone_toggled(d_id: int, toggled_on: bool) -> void:
	if grid_manager != null:
		grid_manager.set_drone_trail_visible(d_id, toggled_on)

	if _is_updating_toggles:
		return

	# Update the Master "All Trails" checkbox state without cascading back
	_is_updating_toggles = true
	var all_checked = true
	for id in created_drone_checkboxes:
		var cb: CheckBox = created_drone_checkboxes[id]
		if is_instance_valid(cb) and not cb.button_pressed:
			all_checked = false
			break

	if master_all_checkbox != null:
		master_all_checkbox.button_pressed = all_checked
	_is_updating_toggles = false

func _on_all_trails_toggled(toggled_on: bool) -> void:
	if _is_updating_toggles:
		return

	if grid_manager != null:
		grid_manager.set_all_trails_visible(toggled_on)

	_is_updating_toggles = true
	for d_id in created_drone_checkboxes:
		var cb: CheckBox = created_drone_checkboxes[d_id]
		if is_instance_valid(cb):
			cb.button_pressed = toggled_on
	_is_updating_toggles = false
