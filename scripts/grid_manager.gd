extends Node3D

@export_group("Grid Dimensions")
@export var grid_size: Vector3i = Vector3i(30, 30, 30)
@export var cell_size: float = 1.0
@export var sensor_radius: float = 3.0
@export var octree_step_size: int = 3

@export_group("Boundary Visualization")
@export var show_boundary_lines: bool = true
@export var boundary_color: Color = Color(0.2, 0.8, 1.0, 0.8) # Cyan wireframe
@export var boundary_line_width: float = 2.0

# Cell State Dictionaries (Key: Vector3i, Value: bool)
var visited_cells: Dictionary = {}
var obstacle_cells: Dictionary = {}
var blocked_cells: Dictionary = {}

var total_traversable_cells: int = 0
var _total_grid_volume: int = 0
var _boundary_mesh_instance: MeshInstance3D = null

func _ready() -> void:
	_total_grid_volume = grid_size.x * grid_size.y * grid_size.z
	total_traversable_cells = _total_grid_volume

	_create_boundary_lines()
	reset_grid()

func _physics_process(_delta: float) -> void:
	# MULTI-AGENT: Sweeps and marks coverage for EVERY active drone in the swarm
	var drones = get_tree().get_nodes_in_group("drones")
	for drone in drones:
		if is_instance_valid(drone) and not drone.is_queued_for_deletion():
			_mark_visited_around(drone.global_position)

# =====================================================
# BOUNDARY LINE RENDERING (WIREFRAME CUBE)
# =====================================================
func _create_boundary_lines() -> void:
	if not show_boundary_lines:
		return

	if is_instance_valid(_boundary_mesh_instance):
		_boundary_mesh_instance.queue_free()

	_boundary_mesh_instance = MeshInstance3D.new()
	_boundary_mesh_instance.name = "GridBoundaryVisual"
	add_child(_boundary_mesh_instance)

	# Set top-level = false and global_position to (0,0,0) so lines are in world space
	_boundary_mesh_instance.global_position = global_position

	var mat = StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(0.0, 1.0, 1.0, 1.0) # Solid Cyan
	mat.no_depth_test = true # Prevents floor/objects from hiding the boundary lines
	mat.render_priority = 2

	var imm_mesh = ImmediateMesh.new()
	imm_mesh.surface_begin(Mesh.PRIMITIVE_LINES)

	var w = float(grid_size.x) * cell_size
	var h = float(grid_size.y) * cell_size
	var d = float(grid_size.z) * cell_size

	var v0 = Vector3(0, 0, 0); var v1 = Vector3(w, 0, 0)
	var v2 = Vector3(w, 0, d); var v3 = Vector3(0, 0, d)
	var v4 = Vector3(0, h, 0); var v5 = Vector3(w, h, 0)
	var v6 = Vector3(w, h, d); var v7 = Vector3(0, h, d)

	# Bottom square
	imm_mesh.surface_add_vertex(v0); imm_mesh.surface_add_vertex(v1)
	imm_mesh.surface_add_vertex(v1); imm_mesh.surface_add_vertex(v2)
	imm_mesh.surface_add_vertex(v2); imm_mesh.surface_add_vertex(v3)
	imm_mesh.surface_add_vertex(v3); imm_mesh.surface_add_vertex(v0)

	# Top square
	imm_mesh.surface_add_vertex(v4); imm_mesh.surface_add_vertex(v5)
	imm_mesh.surface_add_vertex(v5); imm_mesh.surface_add_vertex(v6)
	imm_mesh.surface_add_vertex(v6); imm_mesh.surface_add_vertex(v7)
	imm_mesh.surface_add_vertex(v7); imm_mesh.surface_add_vertex(v4)

	# Vertical pillars
	imm_mesh.surface_add_vertex(v0); imm_mesh.surface_add_vertex(v4)
	imm_mesh.surface_add_vertex(v1); imm_mesh.surface_add_vertex(v5)
	imm_mesh.surface_add_vertex(v2); imm_mesh.surface_add_vertex(v6)
	imm_mesh.surface_add_vertex(v3); imm_mesh.surface_add_vertex(v7)

	imm_mesh.surface_end()

	_boundary_mesh_instance.mesh = imm_mesh
	_boundary_mesh_instance.material_override = mat # MUST BE material_override in Godot 4
# =====================================================
# PATHFINDING (3D A* ALGORITHM)
# =====================================================

func find_path(start_input, target_input) -> Array[Vector3]:
	var start_coord: Vector3i = world_to_grid(start_input) if start_input is Vector3 else start_input
	var target_coord: Vector3i = world_to_grid(target_input) if target_input is Vector3 else target_input

	var path: Array[Vector3] = []

	if not is_within_bounds(start_coord) or not is_within_bounds(target_coord):
		return path

	if start_coord == target_coord:
		path.append(grid_to_world(target_coord))
		return path

	# If the exact target cell is inside an obstacle/NFZ, redirect to closest free adjacent cell
	if not is_cell_strictly_free(target_coord):
		target_coord = _find_nearest_free_neighbor(target_coord)
		if target_coord == Vector3i(-1, -1, -1):
			return path

	var open_set: Array[Vector3i] = [start_coord]
	var came_from: Dictionary = {}

	var g_score: Dictionary = { start_coord: 0.0 }
	var f_score: Dictionary = { start_coord: _heuristic(start_coord, target_coord) }

	const NEIGHBORS = [
		Vector3i(1,0,0), Vector3i(-1,0,0),
		Vector3i(0,1,0), Vector3i(0,-1,0),
		Vector3i(0,0,1), Vector3i(0,0,-1),
		Vector3i(1,1,0), Vector3i(-1,1,0), Vector3i(1,-1,0), Vector3i(-1,-1,0),
		Vector3i(1,0,1), Vector3i(-1,0,1), Vector3i(1,0,-1), Vector3i(-1,0,-1),
		Vector3i(0,1,1), Vector3i(0,-1,1), Vector3i(0,1,-1), Vector3i(0,-1,-1)
	]

	var max_iterations = 2500
	var iterations = 0

	while not open_set.is_empty() and iterations < max_iterations:
		iterations += 1

		var current = open_set[0]
		var lowest_f = f_score.get(current, INF)
		var current_idx = 0

		for i in range(1, open_set.size()):
			var node = open_set[i]
			var f = f_score.get(node, INF)
			if f < lowest_f:
				lowest_f = f
				current = node
				current_idx = i

		if current == target_coord:
			return _reconstruct_path(came_from, current)

		open_set.remove_at(current_idx)

		for offset in NEIGHBORS:
			var neighbor = current + offset
			if not is_cell_strictly_free(neighbor):
				continue

			var move_cost = sqrt(float(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z))
			var tentative_g = g_score.get(current, INF) + move_cost

			if tentative_g < g_score.get(neighbor, INF):
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g
				f_score[neighbor] = tentative_g + _heuristic(neighbor, target_coord)

				if not open_set.has(neighbor):
					open_set.append(neighbor)

	# Direct path fallback if search exhausted
	path.append(grid_to_world(target_coord))
	return path

func _heuristic(a: Vector3i, b: Vector3i) -> float:
	return sqrt(float((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2))

func _reconstruct_path(came_from: Dictionary, current: Vector3i) -> Array[Vector3]:
	var total_path: Array[Vector3] = [grid_to_world(current)]
	while came_from.has(current):
		current = came_from[current]
		total_path.append(grid_to_world(current))
	total_path.reverse()
	return total_path

func _find_nearest_free_neighbor(coord: Vector3i) -> Vector3i:
	const DIRS = [
		Vector3i(1,0,0), Vector3i(-1,0,0),
		Vector3i(0,1,0), Vector3i(0,-1,0),
		Vector3i(0,0,1), Vector3i(0,0,-1)
	]
	for d in DIRS:
		var n = coord + d
		if is_cell_strictly_free(n):
			return n
	return Vector3i(-1, -1, -1)

# =====================================================
# COVERAGE & VISITATION
# =====================================================

func _mark_visited_around(world_pos: Vector3) -> void:
	var center_coord = world_to_grid(world_pos)
	var radius_in_cells = int(ceil(sensor_radius / cell_size))

	for dx in range(-radius_in_cells, radius_in_cells + 1):
		for dy in range(-radius_in_cells, radius_in_cells + 1):
			for dz in range(-radius_in_cells, radius_in_cells + 1):
				var coord = center_coord + Vector3i(dx, dy, dz)
				if is_within_bounds(coord):
					var cell_world = grid_to_world(coord)
					if world_pos.distance_squared_to(cell_world) <= sensor_radius * sensor_radius:
						if not obstacle_cells.has(coord) and not blocked_cells.has(coord):
							visited_cells[coord] = true

func get_coverage_percentage() -> float:
	if total_traversable_cells <= 0:
		return 0.0
	return (float(visited_cells.size()) / float(total_traversable_cells)) * 100.0

func reset_grid() -> void:
	visited_cells.clear()
	obstacle_cells.clear()
	blocked_cells.clear()

	var root = get_tree().current_scene if get_tree() else null
	if root:
		var nfz_manager = root.get_node_or_null("NoFlyZoneManager")
		if is_instance_valid(nfz_manager) and nfz_manager.has_method("populate_blocked_cells"):
			nfz_manager.populate_blocked_cells(self)

	_update_total_traversable_count()

func _update_total_traversable_count() -> void:
	var blocked_count = blocked_cells.size() + obstacle_cells.size()
	total_traversable_cells = max(1, _total_grid_volume - blocked_count)

# =====================================================
# OBSTACLE & HAZARD REGISTRATION (POLYMORPHIC)
# =====================================================

func register_obstacle(target) -> void:
	if target == null:
		return

	if target is Node3D:
		var pos = target.global_position
		if "radius" in target:
			_register_sphere_obstacle(pos, float(target.radius))
		elif "size" in target and target.size is Vector3:
			_register_box_obstacle(pos, target.size)
		elif target.has_node("CollisionShape3D"):
			var col_shape = target.get_node("CollisionShape3D")
			if is_instance_valid(col_shape) and col_shape.shape is BoxShape3D:
				_register_box_obstacle(pos, col_shape.shape.size)
			elif is_instance_valid(col_shape) and col_shape.shape is SphereShape3D:
				_register_sphere_obstacle(pos, col_shape.shape.radius)
			else:
				_mark_single_cell_obstacle(world_to_grid(pos))
		else:
			_mark_single_cell_obstacle(world_to_grid(pos))

	elif target is Vector3:
		_mark_single_cell_obstacle(world_to_grid(target))

	elif target is Vector3i:
		_mark_single_cell_obstacle(target)

	elif target is Array:
		for item in target:
			register_obstacle(item)

	_update_total_traversable_count()

func unregister_obstacle(target) -> void:
	if target == null:
		return

	if target is Node3D:
		obstacle_cells.erase(world_to_grid(target.global_position))
	elif target is Vector3:
		obstacle_cells.erase(world_to_grid(target))
	elif target is Vector3i:
		obstacle_cells.erase(target)

	_update_total_traversable_count()

func _mark_single_cell_obstacle(coord: Vector3i) -> void:
	if is_within_bounds(coord):
		obstacle_cells[coord] = true
		visited_cells.erase(coord)

func _register_sphere_obstacle(world_pos: Vector3, radius: float) -> void:
	var center = world_to_grid(world_pos)
	var r_cells = int(ceil(radius / cell_size))

	for dx in range(-r_cells, r_cells + 1):
		for dy in range(-r_cells, r_cells + 1):
			for dz in range(-r_cells, r_cells + 1):
				var coord = center + Vector3i(dx, dy, dz)
				if is_within_bounds(coord):
					var cell_pos = grid_to_world(coord)
					if world_pos.distance_squared_to(cell_pos) <= radius * radius:
						_mark_single_cell_obstacle(coord)

func _register_box_obstacle(world_pos: Vector3, box_size: Vector3) -> void:
	var half = box_size * 0.5
	var min_coord = world_to_grid(world_pos - half)
	var max_coord = world_to_grid(world_pos + half)

	for x in range(min_coord.x, max_coord.x + 1):
		for y in range(min_coord.y, max_coord.y + 1):
			for z in range(min_coord.z, max_coord.z + 1):
				_mark_single_cell_obstacle(Vector3i(x, y, z))

func is_cell_strictly_free(coord: Vector3i) -> bool:
	if not is_within_bounds(coord):
		return false
	if obstacle_cells.has(coord) or blocked_cells.has(coord):
		return false
	return true

# =====================================================
# PATH & STEP SAFETY CHECKS
# =====================================================

func is_straight_path_safe(from_coord: Vector3i, to_coord: Vector3i) -> bool:
	if not is_within_bounds(to_coord):
		return false
	return not is_straight_path_hazardous(from_coord, to_coord)

func is_straight_path_hazardous(from_coord: Vector3i, to_coord: Vector3i) -> bool:
	var steps = int(max(abs(to_coord.x - from_coord.x), max(abs(to_coord.y - from_coord.y), abs(to_coord.z - from_coord.z))))
	if steps == 0:
		return not is_cell_strictly_free(from_coord)

	for s in range(steps + 1):
		var t = float(s) / float(steps)
		var check_coord = Vector3i(
			int(round(lerp(float(from_coord.x), float(to_coord.x), t))),
			int(round(lerp(float(from_coord.y), float(to_coord.y), t))),
			int(round(lerp(float(from_coord.z), float(to_coord.z), t)))
		)
		if not is_cell_strictly_free(check_coord):
			return true
	return false

func get_adjacent_octree_center(current_coord: Vector3i, direction_idx: int) -> Vector3i:
	const DIRS = [
		Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
		Vector3i(0, 1, 0), Vector3i(0, -1, 0),
		Vector3i(0, 0, 1), Vector3i(0, 0, -1)
	]
	if direction_idx < 0 or direction_idx >= DIRS.size():
		return current_coord

	var target = current_coord + DIRS[direction_idx] * octree_step_size
	target.x = clampi(target.x, 0, grid_size.x - 1)
	target.y = clampi(target.y, 0, grid_size.y - 1)
	target.z = clampi(target.z, 0, grid_size.z - 1)
	return target

# =====================================================
# FRONTIER CENTROIDS
# =====================================================

func get_frontier_centroids(max_count: int) -> Array[Vector3]:
	var frontiers: Array[Vector3] = []
	if visited_cells.is_empty():
		return frontiers

	var step = 4
	for x in range(0, grid_size.x, step):
		for y in range(0, grid_size.y, step):
			for z in range(0, grid_size.z, step):
				var coord = Vector3i(x, y, z)
				if not visited_cells.has(coord) and is_cell_strictly_free(coord):
					if _has_visited_neighbor(coord):
						frontiers.append(grid_to_world(coord))
						if frontiers.size() >= max_count:
							return frontiers
	return frontiers

func _has_visited_neighbor(coord: Vector3i) -> bool:
	const NEIGHBORS = [
		Vector3i(1,0,0), Vector3i(-1,0,0),
		Vector3i(0,1,0), Vector3i(0,-1,0),
		Vector3i(0,0,1), Vector3i(0,0,-1)
	]
	for n in NEIGHBORS:
		if visited_cells.has(coord + n):
			return true
	return false

# =====================================================
# COORDINATE CONVERSIONS
# =====================================================

func world_to_grid(world_pos: Vector3) -> Vector3i:
	return Vector3i(
		int(floor(world_pos.x / cell_size)),
		int(floor(world_pos.y / cell_size)),
		int(floor(world_pos.z / cell_size))
	)

func grid_to_world(grid_coord: Vector3i) -> Vector3:
	return Vector3(
		(float(grid_coord.x) + 0.5) * cell_size,
		(float(grid_coord.y) + 0.5) * cell_size,
		(float(grid_coord.z) + 0.5) * cell_size
	)

func is_within_bounds(coord: Vector3i) -> bool:
	return coord.x >= 0 and coord.x < grid_size.x and \
		   coord.y >= 0 and coord.y < grid_size.y and \
		   coord.z >= 0 and coord.z < grid_size.z

func is_within_grid_limits(coord: Vector3i) -> bool:
	return is_within_bounds(coord)
