extends Node3D

@export var drone: Node3D

#@group("Grid Settings")
# Adjust this to change how far the drone can see the gray grid around it.
@export var view_radius: int = 10
# Toggle whether the blue trail remains visible permanently across the entire 1000^3 space
@export var keep_trail_visible: bool = true

#@group("Boundary Settings")
# Thickness of the red boundary lines
@export var boundary_thickness: float = 1.0
# Bounding box limits (0 to 1000)
@export var grid_size:int = 100
@export var grid_min: Vector3i = Vector3i(0, 0, 0)
@export var grid_max: Vector3i = Vector3i(grid_size, grid_size, grid_size)

var visited_cells = {}   # Dict of {Vector3i: bool} 
var trail_meshes = {}    # Dict of {Vector3i: MeshInstance3D} for permanent trail boxes
var local_meshes = {}    # Dict of {Vector3i: MeshInstance3D} for sliding gray grid

var gray_material: StandardMaterial3D
var blue_material: StandardMaterial3D
var box_mesh: BoxMesh

var last_drone_grid_pos: Vector3i = Vector3i(99999, 99999, 99999)

func _ready() -> void:
	# Define materials
	gray_material = StandardMaterial3D.new()
	gray_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	gray_material.albedo_color = Color(0.5, 0.5, 0.5, 0.12)
	gray_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	blue_material = StandardMaterial3D.new()
	blue_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	blue_material.albedo_color = Color(0.0, 0.5, 1.0, 0.3)
	blue_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	box_mesh = BoxMesh.new()
	box_mesh.size = Vector3(0.95, 0.95, 0.95)

	# Build physical boundaries and visual grid
	create_boundary_lines()
	preallocate_local_grid()

func preallocate_local_grid() -> void:
	# Pre-allocate local pool of visual meshes
	var diameter = (view_radius * 2) + 1
	for x in range(diameter):
		for y in range(diameter):
			for z in range(diameter):
				var local_offset = Vector3i(x - view_radius, y - view_radius, z - view_radius)
				var mesh_inst = MeshInstance3D.new()
				mesh_inst.mesh = box_mesh
				mesh_inst.material_override = gray_material
				add_child(mesh_inst)
				local_meshes[local_offset] = mesh_inst

func create_boundary_lines() -> void:
	var red_material = StandardMaterial3D.new()
	red_material.albedo_color = Color.RED
	red_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	var size = Vector3(grid_max - grid_min)
	
	# Normalized coordinates representing the 12 edges of a 3D bounding box
	var edges = [
		# X-parallel lines
		[Vector3(0,0,0), Vector3(1,0,0)],
		[Vector3(0,1,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(1,0,1)],
		[Vector3(0,1,1), Vector3(1,1,1)],
		# Y-parallel lines
		[Vector3(0,0,0), Vector3(0,1,0)],
		[Vector3(1,0,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(0,1,1)],
		[Vector3(1,0,1), Vector3(1,1,1)],
		# Z-parallel lines
		[Vector3(0,0,0), Vector3(0,0,1)],
		[Vector3(1,0,0), Vector3(1,0,1)],
		[Vector3(0,1,0), Vector3(0,1,1)],
		[Vector3(1,1,0), Vector3(1,1,1)]
	]

	for edge in edges:
		var p1 = Vector3(
			lerp(float(grid_min.x), float(grid_max.x), edge[0].x),
			lerp(float(grid_min.y), float(grid_max.y), edge[0].y),
			lerp(float(grid_min.z), float(grid_max.z), edge[0].z)
		)
		var p2 = Vector3(
			lerp(float(grid_min.x), float(grid_max.x), edge[1].x),
			lerp(float(grid_min.y), float(grid_max.y), edge[1].y),
			lerp(float(grid_min.z), float(grid_max.z), edge[1].z)
		)

		var edge_mesh = BoxMesh.new()
		var dir = p2 - p1

		# Sizing the boundary beam based on orientation
		if dir.x > 0:
			edge_mesh.size = Vector3(dir.x, boundary_thickness, boundary_thickness)
		elif dir.y > 0:
			edge_mesh.size = Vector3(boundary_thickness, dir.y, boundary_thickness)
		else:
			edge_mesh.size = Vector3(boundary_thickness, boundary_thickness, dir.z)

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

	if is_within_bounds(drone_grid_pos):
		if not visited_cells.has(drone_grid_pos):
			visited_cells[drone_grid_pos] = true
			if keep_trail_visible:
				spawn_permanent_trail_box(drone_grid_pos)

	if drone_grid_pos != last_drone_grid_pos:
		update_local_grid(drone_grid_pos)
		last_drone_grid_pos = drone_grid_pos

func spawn_permanent_trail_box(coord: Vector3i) -> void:
	var mesh_inst = MeshInstance3D.new()
	mesh_inst.mesh = box_mesh
	mesh_inst.material_override = blue_material
	mesh_inst.global_position = Vector3(coord.x + 0.5, coord.y + 0.5, coord.z + 0.5)
	add_child(mesh_inst)
	trail_meshes[coord] = mesh_inst

func update_local_grid(center_pos: Vector3i) -> void:
	for local_offset in local_meshes.keys():
		var mesh_inst = local_meshes[local_offset]
		var world_coord = center_pos + local_offset

		if is_within_bounds(world_coord):
			# If the permanent trail is active and this spot is visited, we hide the local 
			# mesh completely to prevent duplicate rendering (z-fighting)
			if keep_trail_visible and visited_cells.has(world_coord):
				mesh_inst.visible = false
			else:
				mesh_inst.visible = true
				mesh_inst.global_position = Vector3(world_coord.x + 0.5, world_coord.y + 0.5, world_coord.z + 0.5)
				
				if visited_cells.has(world_coord):
					mesh_inst.material_override = blue_material
				else:
					mesh_inst.material_override = gray_material
		else:
			mesh_inst.visible = false

func is_within_bounds(coord: Vector3i) -> bool:
	return (coord.x >= grid_min.x and coord.x < grid_max.x and
			coord.y >= grid_min.y and coord.y < grid_max.y and
			coord.z >= grid_min.z and coord.z < grid_max.z)

func _find_drone() -> void:
	var drones = get_tree().get_nodes_in_group("drone")
	if drones.size() > 0:
		drone = drones[0]
