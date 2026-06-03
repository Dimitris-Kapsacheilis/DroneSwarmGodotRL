class_name NoFlyZone
extends Node3D

var polygon: Array[Vector2] = []
var min_altitude: float = 0.0
var max_altitude: float = 100.0
var drone_inside := false

var mesh_instance: MeshInstance3D
var box: MeshInstance3D
var im: ImmediateMesh
var mat: StandardMaterial3D
var box_mat: StandardMaterial3D

func _ready():
	# 1. Initialize the node components and materials with default settings.
	# We do not build the geometry here yet because we don't have the setup data.
	mesh_instance = MeshInstance3D.new()
	add_child(mesh_instance)
	
	im = ImmediateMesh.new()
	mesh_instance.mesh = im
	
	mat = StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(0, 1, 0, 0.8) # Default safe green
	mesh_instance.material_override = mat
	
	box = MeshInstance3D.new()
	add_child(box)
	
	box_mat = StandardMaterial3D.new()
	box_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	box_mat.albedo_color = Color(0, 1, 0, 0.05) # Safe green, semi-transparent
	box_mat.cull_mode = BaseMaterial3D.CULL_DISABLED 
	box.material_override = box_mat

func setup(
	p_polygon: Array[Vector2],
	p_min_altitude: float,
	p_max_altitude: float
):
	polygon = p_polygon
	min_altitude = p_min_altitude
	max_altitude = p_max_altitude
	
	# Create the visual elements now that we have the custom coordinates
	_update_visual_geometry()

func _update_visual_geometry():
	if polygon.size() < 3:
		return

	# --- 1. Calculate and Position the Visual Box ---
	var min_x = INF
	var max_x = -INF
	var min_z = INF
	var max_z = -INF
	
	for p in polygon:
		min_x = min(min_x, p.x)
		max_x = max(max_x, p.x)
		min_z = min(min_z, p.y) # Vector2.y maps to 3D Z-axis
		max_z = max(max_z, p.y)
		
	var size_x = max_x - min_x
	var size_y = max_altitude - min_altitude
	var size_z = max_z - min_z
	
	var center_x = (min_x + max_x) * 0.5
	var center_y = (min_altitude + max_altitude) * 0.5
	var center_z = (min_z + max_z) * 0.5
	
	var cube = BoxMesh.new()
	cube.size = Vector3(size_x, size_y, size_z)
	box.mesh = cube
	box.position = Vector3(center_x, center_y, center_z)

	# --- 2. Draw the 3D Outline (Run once on setup instead of every frame) ---
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
		
		# Draw bottom ring
		im.surface_add_vertex(a_bottom)
		im.surface_add_vertex(b_bottom)
		
		# Draw top ring
		im.surface_add_vertex(a_top)
		im.surface_add_vertex(b_top)
		
		# Draw vertical columns
		im.surface_add_vertex(a_bottom)
		im.surface_add_vertex(a_top)
	im.surface_end()

func contains_position(pos: Vector3) -> bool:
	if pos.y < min_altitude or pos.y > max_altitude:
		return false
	var point_2d := Vector2(pos.x, pos.z)
	return _point_in_polygon(point_2d)

func update_drone_state(pos: Vector3):
	drone_inside = contains_position(pos)
	if drone_inside:
		# Violated state: Solid Red outlines, semi-transparent Red fill
		mat.albedo_color = Color(1, 0, 0, 1)
		box_mat.albedo_color = Color(1, 0, 0, 0.6)
	else:
		# Safe state: Semi-transparent Green outlines and fill
		mat.albedo_color = Color(0, 1, 0, 1)
		box_mat.albedo_color = Color(0, 1, 0, 0.4)

func _point_in_polygon(point: Vector2) -> bool:
	var inside := false
	var count := polygon.size()
	if count < 3:
		return false
	var j := count - 1
	for i in range(count):
		var pi := polygon[i]
		var pj := polygon[j]
		var intersect := (
			(pi.y > point.y) != (pj.y > point.y)
			and
			point.x <
			(pj.x - pi.x) *
			(point.y - pi.y) /
			(pj.y - pi.y + 0.000001)
			+ pi.x
		)
		if intersect:
			inside = !inside
		j = i
	return inside




func _process(delta):
	im.clear_surfaces()
	im.surface_begin(Mesh.PRIMITIVE_LINES)
	var n = polygon.size()
	if n < 2:
		return
	for i in range(n):
		var a2 = polygon[i]
		var b2 = polygon[(i + 1) % n]
		var a_bottom = Vector3(a2.x, min_altitude, a2.y)
		var b_bottom = Vector3(b2.x, min_altitude, b2.y)
		var a_top = Vector3(a2.x, max_altitude, a2.y)
		var b_top = Vector3(b2.x, max_altitude, b2.y)
		# bottom loop
		im.surface_add_vertex(a_bottom)
		im.surface_add_vertex(b_bottom)
		# top loop
		im.surface_add_vertex(a_top)
		im.surface_add_vertex(b_top)
		# vertical edges
		im.surface_add_vertex(a_bottom)
		im.surface_add_vertex(a_top)
	im.surface_end()
