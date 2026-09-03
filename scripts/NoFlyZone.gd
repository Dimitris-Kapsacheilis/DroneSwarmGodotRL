class_name NoFlyZone
extends Node3D

var polygon: Array[Vector2] = []
var min_altitude: float = 0.0
var max_altitude: float = 100.0
var drone_inside := false

var mesh_instance: MeshInstance3D = null
var box: MeshInstance3D = null
var im: ImmediateMesh = null
var mat: StandardMaterial3D = null
var box_mat: StandardMaterial3D = null

func _ready() -> void:
	_ensure_components_initialized()

# Ensures all mesh and material objects exist before setup assigns to them
func _ensure_components_initialized() -> void:
	if DisplayServer.get_name() == "headless":
		return
	if mesh_instance == null:
		mesh_instance = MeshInstance3D.new()
		mesh_instance.name = "OutlineMesh"
		add_child(mesh_instance)

	if im == null:
		im = ImmediateMesh.new()
		mesh_instance.mesh = im

	if mat == null:
		mat = StandardMaterial3D.new()
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		mat.albedo_color = Color(1.0, 0.2, 0.2, 0.9) # Red outline for NFZ
		mesh_instance.material_override = mat

	if box == null:
		box = MeshInstance3D.new()
		box.name = "VolumeBox"
		add_child(box)

	if box_mat == null:
		box_mat = StandardMaterial3D.new()
		box_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		box_mat.albedo_color = Color(1.0, 0.0, 0.0, 0.15) # Red semi-transparent fill
		box_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		box.material_override = box_mat

func setup(
	p_polygon: Array[Vector2],
	p_min_altitude: float,
	p_max_altitude: float
) -> void:
	_ensure_components_initialized()

	polygon = p_polygon
	min_altitude = p_min_altitude
	max_altitude = p_max_altitude

	_update_visual_geometry()

func _update_visual_geometry() -> void:
	if DisplayServer.get_name() == "headless":
		return
	if polygon.size() < 3:
		return

	_ensure_components_initialized()

	# --- 1. Calculate and Position the Visual Box ---
	var min_x = INF; var max_x = -INF
	var min_z = INF; var max_z = -INF

	for p in polygon:
		min_x = minf(min_x, p.x)
		max_x = maxf(max_x, p.x)
		min_z = minf(min_z, p.y)
		max_z = maxf(max_z, p.y)

	var size_x = max_x - min_x
	var size_y = max_altitude - min_altitude
	var size_z = max_z - min_z

	var center_x = (min_x + max_x) * 0.5
	var center_y = (min_altitude + max_altitude) * 0.5
	var center_z = (min_z + max_z) * 0.5

	var cube = BoxMesh.new()
	cube.size = Vector3(size_x, size_y, size_z)
	box.mesh = cube
	box.global_position = Vector3(center_x, center_y, center_z)

	# --- 2. Draw the 3D Outline ---
	im.clear_surfaces()
	im.surface_begin(Mesh.PRIMITIVE_LINES)
	var n = polygon.size()
	for i in range(n):
		var a2 = polygon[i]
		var b2 = polygon[(i + 1) % n]

		var a_bottom = Vector3(a2.x, min_altitude, a2.y)
		var b_bottom = Vector3(b2.x, min_altitude, b2.y)
		var a_top = Vector3(a2.x, max_altitude, a2.y)
		var b_top = Vector3(b2.x, max_altitude, b2.y)

		# Bottom ring
		im.surface_add_vertex(a_bottom); im.surface_add_vertex(b_bottom)
		# Top ring
		im.surface_add_vertex(a_top); im.surface_add_vertex(b_top)
		# Vertical columns
		im.surface_add_vertex(a_bottom); im.surface_add_vertex(a_top)

	im.surface_end()

func contains_position(pos: Vector3) -> bool:
	if pos.y < min_altitude or pos.y > max_altitude:
		return false
	var point_2d = Vector2(pos.x, pos.z)
	# Fast C++ polygon test in Godot
	return Geometry2D.is_point_in_polygon(point_2d, polygon)

func update_drone_state(pos: Vector3) -> void:
	drone_inside = contains_position(pos)
	if DisplayServer.get_name() == "headless":
		return
	if drone_inside:
		mat.albedo_color = Color(1, 0, 0, 1.0)
		box_mat.albedo_color = Color(1, 0, 0, 0.4)
	else:
		mat.albedo_color = Color(1.0, 0.2, 0.2, 0.8)
		box_mat.albedo_color = Color(1.0, 0.0, 0.0, 0.1)
