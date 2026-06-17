extends Node3D

@export var drone: Node3D

@export var grid_size: Vector3i = Vector3i(100, 100, 100)
@export var yellow_radius: int = 5
@export var camera_fov: float = 90.0
@export var boundary_thickness: float = 2.0

var visited_cells = {}   
var trail_meshes = {}    
var yellow_meshes = {}   

var blue_material: StandardMaterial3D
var yellow_material: StandardMaterial3D
var box_mesh: BoxMesh

var last_drone_grid_pos: Vector3i = Vector3i(99999, 99999, 99999)
var last_drone_forward: Vector3 = Vector3.ZERO
var total_cells_count: float = 0.0

func _ready() -> void:
	total_cells_count = float(grid_size.x) * float(grid_size.y) * float(grid_size.z)

	blue_material = StandardMaterial3D.new()
	blue_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	blue_material.albedo_color = Color(0.0, 0.5, 1.0, 0.3)
	blue_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	yellow_material = StandardMaterial3D.new()
	yellow_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	yellow_material.albedo_color = Color(1.0, 0.85, 0.0, 0.3)
	yellow_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	box_mesh = BoxMesh.new()
	box_mesh.size = Vector3(0.95, 0.95, 0.95)

	create_boundary_lines()
	preallocate_yellow_grid()
	print_coverage_stats()

# Clears the environment state on reset
func reset_grid() -> void:
	visited_cells.clear()
	
	# Delete all physical blue boxes from the scene
	for coord in trail_meshes.keys():
		var mesh_inst = trail_meshes[coord]
		if is_instance_valid(mesh_inst):
			mesh_inst.queue_free()
	trail_meshes.clear()
	
	# Hide all yellow sensor meshes
	for mesh_inst in yellow_meshes.values():
		mesh_inst.visible = false
		
	last_drone_grid_pos = Vector3i(99999, 99999, 99999)
	last_drone_forward = Vector3.ZERO
	print("GridManager: Map has been reset for new episode.")

func preallocate_yellow_grid() -> void:
	var diameter = (yellow_radius * 2) + 1
	for x in range(diameter):
		for y in range(diameter):
			for z in range(diameter):
				var local_offset = Vector3i(x - yellow_radius, y - yellow_radius, z - yellow_radius)
				var mesh_inst = MeshInstance3D.new()
				mesh_inst.mesh = box_mesh
				mesh_inst.material_override = yellow_material
				mesh_inst.visible = false
				add_child(mesh_inst)
				yellow_meshes[local_offset] = mesh_inst

func create_boundary_lines() -> void:
	var red_material = StandardMaterial3D.new()
	red_material.albedo_color = Color.RED
	red_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	var thickness = boundary_thickness
	
	var edges = [
		[Vector3(0,0,0), Vector3(1,0,0)],
		[Vector3(0,1,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(1,0,1)],
		[Vector3(0,1,1), Vector3(1,1,1)],
		[Vector3(0,0,0), Vector3(0,1,0)],
		[Vector3(1,0,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(0,1,1)],
		[Vector3(1,0,1), Vector3(1,1,1)],
		[Vector3(0,0,0), Vector3(0,0,1)],
		[Vector3(1,0,0), Vector3(1,0,1)],
		[Vector3(0,1,0), Vector3(0,1,1)],
		[Vector3(1,1,0), Vector3(1,1,1)]
	]

	for edge in edges:
		var p1 = Vector3(
			float(grid_size.x) * edge[0].x,
			float(grid_size.y) * edge[0].y,
			float(grid_size.z) * edge[0].z
		)
		var p2 = Vector3(
			float(grid_size.x) * edge[1].x,
			float(grid_size.y) * edge[1].y,
			float(grid_size.z) * edge[1].z
		)

		var edge_mesh = BoxMesh.new()
		var dir = p2 - p1

		if dir.x > 0:
			edge_mesh.size = Vector3(dir.x, thickness, thickness)
		elif dir.y > 0:
			edge_mesh.size = Vector3(thickness, dir.y, thickness)
		else:
			edge_mesh.size = Vector3(thickness, thickness, dir.z)

		var mesh_inst = MeshInstance3D.new()
		mesh_inst.mesh = edge_mesh
		mesh_inst.material_override = red_material
		mesh_inst.global_position = (p1 + p2) * 0.5
		add_child(mesh_inst)

func _process(_delta: float) -> void:
	if not is_instance_valid(drone):
		_find_drone()
		return

	var drone_grid_pos = Vector3i(
		floor(drone.global_position.x),
		floor(drone.global_position.y),
		floor(drone.global_position.z)
	)

	var forward_dir = drone.global_transform.basis.z.normalized()

	var moved = drone_grid_pos != last_drone_grid_pos
	var rotated = forward_dir.dot(last_drone_forward) < 0.999

	if moved or rotated:
		mark_yellow_zone_as_visited(drone_grid_pos, forward_dir)
		update_yellow_grid(drone_grid_pos, forward_dir)
		
		last_drone_grid_pos = drone_grid_pos
		last_drone_forward = forward_dir

func is_inside_camera_frustum(local_offset: Vector3i, forward_dir: Vector3, threshold: float) -> bool:
	if Vector3(local_offset).length() > yellow_radius:
		return false
		
	if local_offset == Vector3i.ZERO:
		return true

	var to_cell = Vector3(local_offset).normalized()
	var cos_angle = forward_dir.dot(to_cell)
	
	return cos_angle >= threshold

func mark_yellow_zone_as_visited(center_pos: Vector3i, forward_dir: Vector3) -> void:
	var fov_threshold = cos(deg_to_rad(camera_fov / 2.0))
	var diameter = (yellow_radius * 2) + 1
	var newly_visited = false
	
	for x in range(diameter):
		for y in range(diameter):
			for z in range(diameter):
				var offset = Vector3i(x - yellow_radius, y - yellow_radius, z - yellow_radius)
				var world_coord = center_pos + offset
				
				if is_within_bounds(world_coord):
					if is_inside_camera_frustum(offset, forward_dir, fov_threshold):
						if not visited_cells.has(world_coord):
							visited_cells[world_coord] = true
							spawn_permanent_trail_box(world_coord)
							newly_visited = true

	if newly_visited:
		print_coverage_stats()

func spawn_permanent_trail_box(coord: Vector3i) -> void:
	var mesh_inst = MeshInstance3D.new()
	mesh_inst.mesh = box_mesh
	mesh_inst.material_override = blue_material
	mesh_inst.position = Vector3(coord.x + 0.5, coord.y + 0.5, coord.z + 0.5)
	mesh_inst.visible = false 
	add_child(mesh_inst)
	trail_meshes[coord] = mesh_inst

func update_yellow_grid(center_pos: Vector3i, forward_dir: Vector3) -> void:
	for coord in trail_meshes:
		trail_meshes[coord].visible = true

	var fov_threshold = cos(deg_to_rad(camera_fov / 2.0))

	for local_offset in yellow_meshes.keys():
		var mesh_inst = yellow_meshes[local_offset]
		var world_coord = center_pos + local_offset

		if is_within_bounds(world_coord) and is_inside_camera_frustum(local_offset, forward_dir, fov_threshold):
			mesh_inst.visible = true
			mesh_inst.position = Vector3(world_coord.x + 0.5, world_coord.y + 0.5, world_coord.z + 0.5)

			if trail_meshes.has(world_coord):
				trail_meshes[world_coord].visible = false
		else:
			mesh_inst.visible = false

func get_coverage_percentage() -> float:
	if total_cells_count <= 0.0:
		return 0.0
	return (float(visited_cells.size()) / total_cells_count) * 100.0

func print_coverage_stats() -> void:
	var percentage = get_coverage_percentage()
	var visited_count = visited_cells.size()
	print("Coverage: %.4f%% | Cells Visited: %d / %d" % [percentage, visited_count, int(total_cells_count)])

func is_within_bounds(coord: Vector3i) -> bool:
	return (coord.x >= 0 and coord.x < grid_size.x and
			coord.y >= 0 and coord.y < grid_size.y and
			coord.z >= 0 and coord.z < grid_size.z)

func _find_drone() -> void:
	var drones = get_tree().get_nodes_in_group("drone")
	if drones.size() > 0:
		drone = drones[0]
