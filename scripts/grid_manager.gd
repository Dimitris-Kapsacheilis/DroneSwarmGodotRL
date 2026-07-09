extends Node3D

@export var drone: Node3D

@export var grid_size: Vector3i = Vector3i(30, 30, 30)
@export var yellow_radius: int = 5
@export var camera_fov: float = 90.0
@export var boundary_thickness: float = 2.0
@export var local_search_radius: int = 11 # Limits the WFD search window around the drone

var visited_cells = {}   
var trail_meshes = {}    
var yellow_meshes = {}   

var blue_material: StandardMaterial3D
var yellow_material: StandardMaterial3D
var box_mesh: BoxMesh

var last_drone_grid_pos: Vector3i = Vector3i(99999, 99999, 99999)
var last_drone_forward: Vector3 = Vector3.ZERO
var total_cells_count: float = 0.0

# Directions for 3D neighbors (6-connectivity: Up, Down, Left, Right, Forward, Back)
const DIRECTIONS_3D = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1)
]

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
	#preallocate_yellow_grid()
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
		mesh_inst.position = (p1 + p2) * 0.5
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
		#update_yellow_grid(drone_grid_pos, forward_dir)
		
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
							#spawn_permanent_trail_box(world_coord)   # ACTIVATE THIS FOR BLUE TRAIL FOR VISITED NODES !!!!!!!
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
	#print_rich("[color=yellow]Percentage:[/color] [b][color=cyan]%.2f%%[/color][/b]" % percentage)

func is_within_bounds(coord: Vector3i) -> bool:
	return (coord.x >= 0 and coord.x < grid_size.x and
			coord.y >= 0 and coord.y < grid_size.y and
			coord.z >= 0 and coord.z < grid_size.z)

## Evaluates whether a coordinate is within both the global grid bounds 
## and the local bounding box centered on the drone.
func is_within_local_bounds(coord: Vector3i, center_pos: Vector3i) -> bool:
	return (
		coord.x >= 0 and coord.x < grid_size.x and
		coord.y >= 0 and coord.y < grid_size.y and
		coord.z >= 0 and coord.z < grid_size.z and
		abs(coord.x - center_pos.x) <= local_search_radius and
		abs(coord.y - center_pos.y) <= local_search_radius and
		abs(coord.z - center_pos.z) <= local_search_radius
	)

func _find_drone() -> void:
	var drones = get_tree().get_nodes_in_group("drone")
	if drones.size() > 0:
		drone = drones[0]

# ==============================================================================
# WAVEFRONT FRONTIER DETECTOR (WFD) EXTENSION
# ==============================================================================

## Checks if a coordinate is a frontier cell.
## A cell is a frontier if it has been visited (known free) and has at least 
## one unvisited neighbor within the bounds of the grid.
func is_frontier_cell(coord: Vector3i) -> bool:
	if not is_within_bounds(coord) or not visited_cells.has(coord):
		return false
		
	for dir in DIRECTIONS_3D:
		var neighbor = coord + dir
		if is_within_bounds(neighbor) and not visited_cells.has(neighbor):
			return true
	return false

## Runs the WFD algorithm restricted to a local bounding box around the drone's position.
## Returns an Array of Arrays, where each inner Array represents a cluster of frontier Vector3i coordinates.
func find_frontiers(min_frontier_size: int = 3) -> Array[Array]:
	var detected_frontiers: Array[Array] = []
	
	if not is_instance_valid(drone):
		return detected_frontiers

	var start_pos = Vector3i(
		floor(drone.global_position.x),
		floor(drone.global_position.y),
		floor(drone.global_position.z)
	)

	# Safety fallback: if the drone hasn't marked its own starting point yet,
	# we cannot safely evaluate the wavefront BFS from it.
	if not visited_cells.has(start_pos):
		return detected_frontiers

	var visited_m = {} # Map search visited tracker
	var visited_f = {} # Frontier search visited tracker

	# Map Queue (Queue M) with pointer index to avoid pop_front() allocations
	var queue_m: Array[Vector3i] = [start_pos]
	var head_m: int = 0
	visited_m[start_pos] = true

	while head_m < queue_m.size():
		var p = queue_m[head_m]
		head_m += 1

		if is_frontier_cell(p) and not visited_f.has(p):
			# Found a new frontier point; start localized BFS inside the bounding box
			var queue_f: Array[Vector3i] = [p]
			var head_f: int = 0
			var new_frontier: Array[Vector3i] = []
			
			visited_f[p] = true

			while head_f < queue_f.size():
				var q = queue_f[head_f]
				head_f += 1

				if is_frontier_cell(q):
					new_frontier.append(q)
					
					for dir in DIRECTIONS_3D:
						var w = q + dir
						if is_within_local_bounds(w, start_pos) and not visited_f.has(w):
							if is_frontier_cell(w):
								queue_f.append(w)
								visited_f[w] = true

			# Append cluster if it passes noise threshold filter
			if new_frontier.size() >= min_frontier_size:
				detected_frontiers.append(new_frontier)

			# Mark all points in this cluster as processed in the main map search
			for cell in new_frontier:
				visited_m[cell] = true

		# Expand Map Search BFS to adjacent known free space within the local bounds
		for dir in DIRECTIONS_3D:
			var v = p + dir
			if is_within_local_bounds(v, start_pos) and not visited_m.has(v):
				if visited_cells.has(v): # Must be visited (known free) to propagate wavefront
					queue_m.append(v)
					visited_m[v] = true

	return detected_frontiers

## Extracts the geometric centers (centroids) of all valid frontier groups.
## This is useful for RL observation inputs or target tracking.
func get_frontier_centroids(min_frontier_size: int = 3) -> Array[Vector3]:
	var clusters = find_frontiers(min_frontier_size)
	var centroids: Array[Vector3] = []
	
	for cluster in clusters:
		var sum: Vector3 = Vector3.ZERO
		for cell in cluster:
			# Translate from integer grid coordinate to voxel center position
			sum += Vector3(cell) + Vector3(0.5, 0.5, 0.5)
		centroids.append(sum / float(cluster.size()))
		
	return centroids
