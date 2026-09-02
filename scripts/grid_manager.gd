extends Node3D

@export var drone: Node3D
@export var grid_logger: GridLogger
@export var grid_size: Vector3i = Vector3i(30, 30, 30)
@export var boundary_thickness: float = 0.2
@export var local_search_radius: int = 6

# Safety Clearance Margin
@export var obstacle_safety_margin: float = 2.0

# ---------------------------------------------------------------------------
# Drone Sensor & Visibility Settings
# ---------------------------------------------------------------------------
@export_group("Drone Visibility & FOV")
@export var cone_fov: bool = false
@export var cone_radius: int = 5
@export var camera_fov: float = 90.0
@export var box_radius: int = 3 # e.g. 3 = 7x7x7, 2 = 5x5x5, 1 = 3x3x3

# ---------------------------------------------------------------------------
# Blocked Cells Enforcement
# ---------------------------------------------------------------------------
@export_group("Obstacle & Safety Settings")
# Intentionally false: this controls MOVEMENT enforcement only (whether
# is_within_bounds()/is_diagonal_move_safe() physically block a move into an
# obstacle/NFZ cell). Keeping it false lets the agent actually attempt unsafe
# moves and experience the real collision consequence, which is necessary if
# you want it to learn avoidance from experience rather than have it hard-
# blocked. Observation accuracy (directional clearance, hazard checks) no
# longer depends on this flag at all — see is_cell_hazardous() /
# is_straight_path_hazardous() below, which always reflect real occupancy
# regardless of this setting. Only flip this to true if you specifically want
# the grid to physically prevent unsafe moves (e.g. for pathfinding tools
# like find_path(), or a masked-training mode).
@export var enforce_blocked_cells: bool = false

var visited_cells = {}
var blocked_cells = {}      # Permanent blocked coordinates (NFZs)
var obstacle_cells = {}     # Temporary blocked coordinates (Spawned obstacles)
var octree_nodes: Array[Dictionary] = [] # All hierarchical Octree groups
var cell_to_octree_idx: Dictionary = {}  # Fast spatial mapping: Vector3i -> Octree Node Index

var trail_meshes = {}
var yellow_meshes = {}
var blue_material: StandardMaterial3D
var yellow_material: StandardMaterial3D
var box_mesh: BoxMesh

var last_drone_grid_pos: Vector3i = Vector3i(99999, 99999, 99999)
var last_drone_forward: Vector3 = Vector3.ZERO
var total_cells_count: float = 0.0
var _summary_pending := false

const DIRECTIONS = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1)
]

func _ready() -> void:
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

	await get_tree().process_frame
	initialize_grid_space()

func initialize_grid_space() -> void:
	blocked_cells.clear()
	var nfz_nodes = _find_no_fly_zones()

	for zone in nfz_nodes:
		if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
			push_error("GridManager Error: NoFlyZone node '%s' missing required variables." % zone.name)

	for x in range(grid_size.x):
		for y in range(grid_size.y):
			for z in range(grid_size.z):
				var coord = Vector3i(x, y, z)
				var cell_center = Vector3(float(x) + 0.5, float(y) + 0.5, float(z) + 0.5)
				for zone in nfz_nodes:
					if _is_point_near_zone_with_margin(cell_center, zone, obstacle_safety_margin):
						blocked_cells[coord] = true
						break

	_build_octree_grid()
	recalculate_total_cells()
	print_coverage_stats()

# =====================================================
# EXACT NON-OVERLAPPING MULTI-RESOLUTION TILING
# =====================================================

func _build_octree_grid() -> void:
	octree_nodes.clear()
	cell_to_octree_idx.clear()

	var max_macro_size = max(1, (box_radius * 2) + 1)

	var candidate_sizes: Array[int] = []
	var curr_size = max_macro_size
	while curr_size >= 1:
		candidate_sizes.append(curr_size)
		curr_size -= 2
	if not candidate_sizes.has(1):
		candidate_sizes.append(1)

	var assigned_cells: Dictionary = {}

	for x in range(grid_size.x):
		for y in range(grid_size.y):
			for z in range(grid_size.z):
				var coord = Vector3i(x, y, z)
				if assigned_cells.has(coord):
					continue

				var best_size = 1
				var is_free_cube = false

				for s in candidate_sizes:
					if s > 1:
						if _can_fit_free_cube(coord, s, assigned_cells):
							best_size = s
							is_free_cube = true
							break
					else:
						best_size = 1
						is_free_cube = not (blocked_cells.has(coord) or obstacle_cells.has(coord))

				var node_idx = octree_nodes.size()
				var node_cells: Array[Vector3i] = []
				var center_cell = coord + Vector3i(best_size / 2, best_size / 2, best_size / 2)

				for dx in range(best_size):
					for dy in range(best_size):
						for dz in range(best_size):
							var c = coord + Vector3i(dx, dy, dz)
							assigned_cells[c] = true
							cell_to_octree_idx[c] = node_idx
							node_cells.append(c)

				octree_nodes.append({
					"origin": coord,
					"size": best_size,
					"center": center_cell,
					"is_blocked": not is_free_cube,
					"cells": node_cells
				})

func _can_fit_free_cube(origin: Vector3i, size: int, assigned_cells: Dictionary) -> bool:
	if origin.x + size > grid_size.x or origin.y + size > grid_size.y or origin.z + size > grid_size.z:
		return false

	for dx in range(size):
		for dy in range(size):
			for dz in range(size):
				var c = origin + Vector3i(dx, dy, dz)
				if assigned_cells.has(c):
					return false
				if blocked_cells.has(c) or obstacle_cells.has(c):
					return false
	return true

func get_adjacent_octree_center(from_grid_pos: Vector3i, dir_idx: int) -> Vector3i:
	if dir_idx < 0 or dir_idx >= DIRECTIONS.size():
		return from_grid_pos

	var dir = DIRECTIONS[dir_idx]
	var current_node_idx = cell_to_octree_idx.get(from_grid_pos, -1)

	var probe_cell: Vector3i
	if current_node_idx != -1:
		var current_node = octree_nodes[current_node_idx]
		var origin = current_node.origin
		var size = current_node.size

		if dir.x > 0: probe_cell = Vector3i(origin.x + size, from_grid_pos.y, from_grid_pos.z)
		elif dir.x < 0: probe_cell = Vector3i(origin.x - 1, from_grid_pos.y, from_grid_pos.z)
		elif dir.y > 0: probe_cell = Vector3i(from_grid_pos.x, origin.y + size, from_grid_pos.z)
		elif dir.y < 0: probe_cell = Vector3i(from_grid_pos.x, origin.y - 1, from_grid_pos.z)
		elif dir.z > 0: probe_cell = Vector3i(from_grid_pos.x, origin.y, origin.z + size)
		else: probe_cell = Vector3i(from_grid_pos.x, origin.y, origin.z - 1)
	else:
		probe_cell = from_grid_pos + dir

	if not is_within_grid_limits(probe_cell):
		return from_grid_pos

	var neighbor_node_idx = cell_to_octree_idx.get(probe_cell, -1)
	if neighbor_node_idx != -1:
		return octree_nodes[neighbor_node_idx].center

	return probe_cell

func world_to_grid(world_pos: Vector3) -> Vector3i:
	return Vector3i(floor(world_pos.x), floor(world_pos.y), floor(world_pos.z))

func is_cell_strictly_free(coord: Vector3i) -> bool:
	if not is_within_grid_limits(coord):
		return false
	if blocked_cells.has(coord) or obstacle_cells.has(coord):
		return false
	return true

# FIX: registered obstacle footprint is now inflated by obstacle_safety_margin,
# matching the treatment NFZs already got via _is_point_near_zone_with_margin.
# Without this, a grid cell right next to an obstacle could read as "free"
# while still being close enough for the drone's own body to clip it.
func register_obstacle(obstacle: Node3D) -> void:
	var shape_node = obstacle.find_child("CollisionShape3D", true, false)
	if not shape_node or not shape_node.shape:
		return

	var shape = shape_node.shape
	var global_center = shape_node.global_position
	var half_extents = Vector3.ZERO
	var node_scale = shape_node.global_transform.basis.get_scale()

	if shape is BoxShape3D:
		half_extents = (shape.size / 2.0) * node_scale
	elif shape is SphereShape3D:
		var radius = shape.radius * node_scale.x
		half_extents = Vector3(radius, radius, radius)
	elif shape is CylinderShape3D:
		var r = shape.radius * node_scale.x
		var h = shape.height * node_scale.y
		half_extents = Vector3(r, h/2.0, r)

	# Inflate by the safety margin so registered obstacle cells account for
	# the drone's own physical size, not just the obstacle's exact geometry.
	half_extents += Vector3.ONE * obstacle_safety_margin

	var min_pos = global_center - half_extents
	var max_pos = global_center + half_extents

	var min_grid = world_to_grid(min_pos)
	var max_grid = world_to_grid(max_pos)

	min_grid.x = clamp(min_grid.x, 0, grid_size.x - 1)
	min_grid.y = clamp(min_grid.y, 0, grid_size.y - 1)
	min_grid.z = clamp(min_grid.z, 0, grid_size.z - 1)
	max_grid.x = clamp(max_grid.x, 0, grid_size.x - 1)
	max_grid.y = clamp(max_grid.y, 0, grid_size.y - 1)
	max_grid.z = clamp(max_grid.z, 0, grid_size.z - 1)

	for x in range(min_grid.x, max_grid.x + 1):
		for y in range(min_grid.y, max_grid.y + 1):
			for z in range(min_grid.z, max_grid.z + 1):
				obstacle_cells[Vector3i(x, y, z)] = true

	_build_octree_grid()
	recalculate_total_cells()

func recalculate_total_cells() -> void:
	var raw_total := grid_size.x * grid_size.y * grid_size.z

	var unique_blocked_cells: Dictionary = {}
	for c in blocked_cells: unique_blocked_cells[c] = true
	for c in obstacle_cells: unique_blocked_cells[c] = true
	var total_blocked_count = unique_blocked_cells.size()

	if not enforce_blocked_cells:
		total_cells_count = float(raw_total)
	else:
		total_cells_count = float(max(raw_total - total_blocked_count, 1))

	if not _summary_pending:
		_summary_pending = true
		_print_summary_deferred.call_deferred()

func _print_summary_deferred() -> void:
	_summary_pending = false
	var raw_total := grid_size.x * grid_size.y * grid_size.z

	var unique_blocked_cells: Dictionary = {}
	for c in blocked_cells: unique_blocked_cells[c] = true
	for c in obstacle_cells: unique_blocked_cells[c] = true
	var total_blocked_count = unique_blocked_cells.size()
	var total_free_count = raw_total - total_blocked_count

	var free_octree_groups = 0
	var blocked_octree_groups = 0
	for node in octree_nodes:
		if node.is_blocked:
			blocked_octree_groups += 1
		else:
			free_octree_groups += 1

	print("GridManager: Reset. Total Cells: %d (Free: %d, Blocked: %d) | Total Octree Groups: %d (Free: %d, Blocked: %d) | FOV: %s | Enforcing blocked cells: %s" % [
		raw_total,
		total_free_count,
		total_blocked_count,
		octree_nodes.size(),
		free_octree_groups,
		blocked_octree_groups,
		"CONE (dist=%d, fov=%.1f)" % [cone_radius, camera_fov] if cone_fov else "BOX (%dx%dx%d)" % [(box_radius*2+1), (box_radius*2+1), (box_radius*2+1)],
		str(enforce_blocked_cells)
	])

# =====================================================
# SENSOR & VISITATION LOGIC
# =====================================================

func mark_yellow_zone_as_visited(center_pos: Vector3i, forward_dir: Vector3) -> void:
	if cone_fov:
		var fov_threshold = cos(deg_to_rad(camera_fov / 2.0))
		var diameter = (cone_radius * 2) + 1

		for x in range(diameter):
			for y in range(diameter):
				for z in range(diameter):
					var offset = Vector3i(x - cone_radius, y - cone_radius, z - cone_radius)
					var world_coord = center_pos + offset

					if is_within_bounds(world_coord):
						if is_inside_camera_frustum(offset, forward_dir, fov_threshold):
							if not visited_cells.has(world_coord):
								visited_cells[world_coord] = true
								if is_instance_valid(grid_logger):
									grid_logger.log_visited(world_coord)
	else:
		var r = max(0, box_radius)
		for x in range(-r, r + 1):
			for y in range(-r, r + 1):
				for z in range(-r, r + 1):
					var world_coord = center_pos + Vector3i(x, y, z)
					if is_within_bounds(world_coord):
						if not visited_cells.has(world_coord):
							visited_cells[world_coord] = true
							if is_instance_valid(grid_logger):
								grid_logger.log_visited(world_coord)

# NOTE ON RESET ORDER: this clears obstacle_cells. If your obstacle spawner
# registers obstacles via register_obstacle(), it MUST do so AFTER this
# function returns, or its registrations will be wiped here. Call order
# should be: reset_grid() -> obstacle_spawner respawns + re-registers.
func reset_grid() -> void:
	if is_instance_valid(grid_logger):
		grid_logger.save_episode_data(get_coverage_percentage(), visited_cells.size())
		grid_logger.clear_episode_data()

	visited_cells.clear()
	obstacle_cells.clear()
	blocked_cells.clear()

	var nfz_nodes = _find_no_fly_zones()
	for x in range(grid_size.x):
		for y in range(grid_size.y):
			for z in range(grid_size.z):
				var coord = Vector3i(x, y, z)
				var cell_center = Vector3(float(x) + 0.5, float(y) + 0.5, float(z) + 0.5)
				for zone in nfz_nodes:
					if _is_point_near_zone_with_margin(cell_center, zone, obstacle_safety_margin):
						blocked_cells[coord] = true
						break

	_build_octree_grid()
	recalculate_total_cells()

	for coord in trail_meshes.keys():
		var mesh_inst = trail_meshes[coord]
		if is_instance_valid(mesh_inst):
			mesh_inst.queue_free()
	trail_meshes.clear()

	for mesh_inst in yellow_meshes.values():
		mesh_inst.visible = false

	last_drone_grid_pos = Vector3i(99999, 99999, 99999)
	last_drone_forward = Vector3.ZERO

func preallocate_yellow_grid() -> void:
	var r = cone_radius if cone_fov else box_radius
	var diameter = (r * 2) + 1
	for x in range(diameter):
		for y in range(diameter):
			for z in range(diameter):
				var local_offset = Vector3i(x - r, y - r, z - r)
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
	# FIX: entry 11 was a degenerate zero-length edge ((0,0,1)-(0,0,1)); this
	# left the top face missing one edge in the rendered boundary box. Purely
	# cosmetic, doesn't affect training.
	var edges = [
		[Vector3(0,0,0), Vector3(1,0,0)], [Vector3(0,1,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(1,0,1)], [Vector3(0,1,1), Vector3(1,1,1)],
		[Vector3(0,0,0), Vector3(0,1,0)], [Vector3(1,0,0), Vector3(1,1,0)],
		[Vector3(0,0,1), Vector3(0,1,1)], [Vector3(1,0,1), Vector3(1,1,1)],
		[Vector3(0,0,0), Vector3(0,0,1)], [Vector3(1,0,0), Vector3(1,0,1)],
		[Vector3(0,1,0), Vector3(0,1,1)], [Vector3(1,1,0), Vector3(1,1,1)]
	]

	for edge in edges:
		var p1 = Vector3(float(grid_size.x) * edge[0].x, float(grid_size.y) * edge[0].y, float(grid_size.z) * edge[0].z)
		var p2 = Vector3(float(grid_size.x) * edge[1].x, float(grid_size.y) * edge[1].y, float(grid_size.z) * edge[1].z)

		var edge_mesh = BoxMesh.new()
		var dir = p2 - p1

		if dir.x > 0: edge_mesh.size = Vector3(dir.x, thickness, thickness)
		elif dir.y > 0: edge_mesh.size = Vector3(thickness, dir.y, thickness)
		else: edge_mesh.size = Vector3(thickness, thickness, dir.z)

		var mesh_inst = MeshInstance3D.new()
		mesh_inst.mesh = edge_mesh
		mesh_inst.material_override = red_material
		mesh_inst.position = (p1 + p2) * 0.5
		add_child(mesh_inst)

func _process(_delta: float) -> void:
	if not is_instance_valid(drone):
		_find_drone()
		return

	var drone_grid_pos = world_to_grid(drone.global_position)
	var forward_dir = drone.global_transform.basis.z.normalized()

	var moved = drone_grid_pos != last_drone_grid_pos
	var rotated = forward_dir.dot(last_drone_forward) < 0.999

	if moved or (cone_fov and rotated):
		mark_yellow_zone_as_visited(drone_grid_pos, forward_dir)
		last_drone_grid_pos = drone_grid_pos
		last_drone_forward = forward_dir

func is_inside_camera_frustum(local_offset: Vector3i, forward_dir: Vector3, threshold: float) -> bool:
	if Vector3(local_offset).length() > cone_radius:
		return false
	if local_offset == Vector3i.ZERO:
		return true
	var to_cell = Vector3(local_offset).normalized()
	var cos_angle = forward_dir.dot(to_cell)
	return cos_angle >= threshold

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
	var r = cone_radius if cone_fov else box_radius

	for local_offset in yellow_meshes.keys():
		var mesh_inst = yellow_meshes[local_offset]
		var world_coord = center_pos + local_offset

		var is_visible = false
		if cone_fov:
			is_visible = is_within_bounds(world_coord) and is_inside_camera_frustum(local_offset, forward_dir, fov_threshold)
		else:
			is_visible = is_within_bounds(world_coord) and (abs(local_offset.x) <= r and abs(local_offset.y) <= r and abs(local_offset.z) <= r)

		if is_visible:
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
	print("Coverage: %.2f%% (%d / %d cells)" % [percentage, visited_count, int(total_cells_count)])

func is_within_bounds(coord: Vector3i) -> bool:
	var inside_grid = (coord.x >= 0 and coord.x < grid_size.x and
		coord.y >= 0 and coord.y < grid_size.y and
		coord.z >= 0 and coord.z < grid_size.z)

	if not inside_grid:
		return false

	if enforce_blocked_cells:
		if blocked_cells.has(coord) or obstacle_cells.has(coord):
			return false

	return true

func is_within_grid_limits(coord: Vector3i) -> bool:
	return (coord.x >= 0 and coord.x < grid_size.x and
		coord.y >= 0 and coord.y < grid_size.y and
		coord.z >= 0 and coord.z < grid_size.z)

func is_within_local_bounds(coord: Vector3i, center_pos: Vector3i) -> bool:
	return (
		is_within_bounds(coord) and
		abs(coord.x - center_pos.x) <= local_search_radius and
		abs(coord.y - center_pos.y) <= local_search_radius and
		abs(coord.z - center_pos.z) <= local_search_radius
	)

func _find_drone() -> void:
	var drones = get_tree().get_nodes_in_group("drone")
	if drones.size() > 0:
		drone = drones[0]

func _find_no_fly_zones() -> Array:
	var found: Array = []
	var root = get_tree().current_scene
	if root:
		_find_no_fly_zones_recursive(root, found)
	return found

func _find_no_fly_zones_recursive(node: Node, found: Array) -> void:
	if node is NoFlyZone:
		found.append(node)
	for child in node.get_children():
		_find_no_fly_zones_recursive(child, found)

func _is_point_inside_zone(point: Vector3, zone: Node) -> bool:
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return false
	if point.y < zone.min_altitude or point.y > zone.max_altitude:
		return false
	var points_array = zone.polygon
	if points_array.size() == 0:
		return false
	var min_x: float = points_array[0].x
	var max_x: float = points_array[0].x
	var min_z: float = points_array[0].y
	var max_z: float = points_array[0].y
	for i in range(1, points_array.size()):
		var p = points_array[i]
		if p.x < min_x: min_x = p.x
		elif p.x > max_x: max_x = p.x
		if p.y < min_z: min_z = p.y
		elif p.y > max_z: max_z = p.y
	return point.x >= min_x and point.x <= max_x and point.z >= min_z and point.z <= max_z

func _is_point_near_zone_with_margin(point: Vector3, zone: Node, margin: float) -> bool:
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return false

	if point.y < (zone.min_altitude - margin) or point.y > (zone.max_altitude + margin):
		return false

	var points_array = zone.polygon
	if points_array.size() == 0:
		return false

	var point_2d = Vector2(point.x, point.z)

	if _is_point_inside_zone(point, zone):
		return true

	var margin_sq = margin * margin
	var n = points_array.size()
	for i in range(n):
		var a = points_array[i]
		var b = points_array[(i + 1) % n]
		if _point_to_segment_distance_sq(point_2d, a, b) <= margin_sq:
			return true

	return false

func _point_to_segment_distance_sq(p: Vector2, a: Vector2, b: Vector2) -> float:
	var ab = b - a
	var ap = p - a
	var l2 = ab.length_squared()
	if l2 == 0.0:
		return p.distance_squared_to(a)
	var t = ap.dot(ab) / l2
	t = clampf(t, 0.0, 1.0)
	var closest = a + ab * t
	return p.distance_squared_to(closest)

func is_frontier_cell(coord: Vector3i) -> bool:
	if not is_within_bounds(coord) or not visited_cells.has(coord):
		return false
	for dir in DIRECTIONS:
		var neighbor = coord + dir
		if is_within_bounds(neighbor) and not visited_cells.has(neighbor):
			return true
	return false

func find_frontiers(min_frontier_size: int = 3) -> Array[Array]:
	var detected_frontiers: Array[Array] = []
	if not is_instance_valid(drone):
		return detected_frontiers

	var start_pos = world_to_grid(drone.global_position)
	if not visited_cells.has(start_pos):
		return detected_frontiers

	var visited_m = {}
	var visited_f = {}
	var queue_m: Array[Vector3i] = [start_pos]
	var head_m: int = 0
	visited_m[start_pos] = true

	while head_m < queue_m.size():
		var p = queue_m[head_m]
		head_m += 1

		if is_frontier_cell(p) and not visited_f.has(p):
			var queue_f: Array[Vector3i] = [p]
			var head_f: int = 0
			var new_frontier: Array[Vector3i] = []
			visited_f[p] = true

			while head_f < queue_f.size():
				var q = queue_f[head_f]
				head_f += 1

				if is_frontier_cell(q):
					new_frontier.append(q)
					for dir in DIRECTIONS:
						var w = q + dir
						if is_within_local_bounds(w, start_pos) and not visited_f.has(w):
							if is_frontier_cell(w):
								queue_f.append(w)
								visited_f[w] = true

			if new_frontier.size() >= min_frontier_size:
				detected_frontiers.append(new_frontier)

			for cell in new_frontier:
				visited_m[cell] = true

		for dir in DIRECTIONS:
			var v = p + dir
			if is_within_local_bounds(v, start_pos) and not visited_m.has(v):
				if visited_cells.has(v):
					queue_m.append(v)
					visited_m[v] = true

	return detected_frontiers

func get_frontier_centroids(min_frontier_size: int = 3) -> Array[Vector3]:
	var clusters = find_frontiers(min_frontier_size)
	var centroids: Array[Vector3] = []

	for cluster in clusters:
		var sum: Vector3 = Vector3.ZERO
		for cell in cluster:
			sum += Vector3(cell) + Vector3(0.5, 0.5, 0.5)
		centroids.append(sum / float(cluster.size()))

	return centroids

func find_path(start: Vector3i, end: Vector3i) -> Array[Vector3i]:
	if not is_within_bounds(start) or not is_within_bounds(end):
		return []

	if start == end:
		return [start]

	var open_set: Array[Vector3i] = [start]
	var came_from: Dictionary = {}

	var g_score: Dictionary = {}
	g_score[start] = 0.0

	var f_score: Dictionary = {}
	f_score[start] = _heuristic(start, end)

	while open_set.size() > 0:
		var current = open_set[0]
		var lowest_f = f_score.get(current, INF)

		for node in open_set:
			var score = f_score.get(node, INF)
			if score < lowest_f:
				current = node
				lowest_f = score

		if current == end:
			return _reconstruct_path(came_from, current)

		open_set.erase(current)

		for dir in DIRECTIONS:
			var neighbor = current + dir
			if not is_within_bounds(neighbor):
				continue
			if not is_diagonal_move_safe(current, neighbor):
				continue

			var move_cost = Vector3(dir).length()
			var tentative_g = g_score[current] + move_cost

			if tentative_g < g_score.get(neighbor, INF):
				came_from[neighbor] = current
				g_score[neighbor] = tentative_g
				f_score[neighbor] = tentative_g + _heuristic(neighbor, end)

				if not open_set.has(neighbor):
					open_set.append(neighbor)

	return []

func _heuristic(a: Vector3i, b: Vector3i) -> float:
	return Vector3(a).distance_to(Vector3(b))

func _reconstruct_path(came_from: Dictionary, current: Vector3i) -> Array[Vector3i]:
	var total_path: Array[Vector3i] = [current]
	while came_from.has(current):
		current = came_from[current]
		total_path.push_front(current)
	return total_path

func is_diagonal_move_safe(from_coord: Vector3i, to_coord: Vector3i) -> bool:
	if not enforce_blocked_cells:
		return true

	var diff = to_coord - from_coord
	var adx = abs(diff.x)
	var ady = abs(diff.y)
	var adz = abs(diff.z)

	var axes_changed = 0
	if adx > 0: axes_changed += 1
	if ady > 0: axes_changed += 1
	if adz > 0: axes_changed += 1

	if axes_changed <= 1:
		return true

	var dx = diff.x
	var dy = diff.y
	var dz = diff.z

	if axes_changed == 2:
		if dx != 0 and dy != 0:
			if blocked_cells.has(from_coord + Vector3i(dx, 0, 0)) or blocked_cells.has(from_coord + Vector3i(0, dy, 0)) or obstacle_cells.has(from_coord + Vector3i(dx, 0, 0)) or obstacle_cells.has(from_coord + Vector3i(0, dy, 0)):
				return false
		elif dx != 0 and dz != 0:
			if blocked_cells.has(from_coord + Vector3i(dx, 0, 0)) or blocked_cells.has(from_coord + Vector3i(0, 0, dz)) or obstacle_cells.has(from_coord + Vector3i(dx, 0, 0)) or obstacle_cells.has(from_coord + Vector3i(0, 0, dz)):
				return false
		elif dy != 0 and dz != 0:
			if blocked_cells.has(from_coord + Vector3i(0, dy, 0)) or blocked_cells.has(from_coord + Vector3i(0, 0, dz)) or obstacle_cells.has(from_coord + Vector3i(0, dy, 0)) or obstacle_cells.has(from_coord + Vector3i(0, 0, dz)):
				return false

	elif axes_changed == 3:
		if blocked_cells.has(from_coord + Vector3i(dx, 0, 0)) or \
		   blocked_cells.has(from_coord + Vector3i(0, dy, 0)) or \
		   blocked_cells.has(from_coord + Vector3i(0, 0, dz)) or \
		   obstacle_cells.has(from_coord + Vector3i(dx, 0, 0)) or \
		   obstacle_cells.has(from_coord + Vector3i(0, dy, 0)) or \
		   obstacle_cells.has(from_coord + Vector3i(0, 0, dz)):
			return false
		if blocked_cells.has(from_coord + Vector3i(dx, dy, 0)) or \
		   blocked_cells.has(from_coord + Vector3i(dx, 0, dz)) or \
		   blocked_cells.has(from_coord + Vector3i(0, dy, dz)) or \
		   obstacle_cells.has(from_coord + Vector3i(dx, dy, 0)) or \
		   obstacle_cells.has(from_coord + Vector3i(dx, 0, dz)) or \
		   obstacle_cells.has(from_coord + Vector3i(0, dy, dz)):
			return false

	return true

func is_straight_path_safe(from_coord: Vector3i, to_coord: Vector3i) -> bool:
	var current = from_coord
	var diff = to_coord - from_coord

	var steps = max(abs(diff.x), max(abs(diff.y), abs(diff.z)))
	if steps == 0:
		return true

	var step_dir = Vector3i(
		roundi(float(diff.x) / steps),
		roundi(float(diff.y) / steps),
		roundi(float(diff.z) / steps)
	)

	for i in range(steps):
		var next_cell = current + step_dir
		if not is_within_bounds(next_cell):
			return false
		if not is_diagonal_move_safe(current, next_cell):
			return false
		current = next_cell

	return true

# =====================================================
# HAZARD AWARENESS (independent of enforce_blocked_cells)
# =====================================================
# These always check real obstacle/NFZ occupancy, regardless of whether
# enforce_blocked_cells is physically blocking movement. Use these for
# anything that needs an accurate danger signal without affecting whether a
# move is allowed to execute — i.e. observations. This is what lets the
# agent actually see "there's an obstacle that way" while still being free
# to fly into it and learn from the consequence, instead of the environment
# silently rerouting it away and depriving it of that experience.

func is_cell_hazardous(coord: Vector3i) -> bool:
	if not is_within_grid_limits(coord):
		return true
	return blocked_cells.has(coord) or obstacle_cells.has(coord)

func is_straight_path_hazardous(from_coord: Vector3i, to_coord: Vector3i) -> bool:
	var current = from_coord
	var diff = to_coord - from_coord

	var steps = max(abs(diff.x), max(abs(diff.y), abs(diff.z)))
	if steps == 0:
		return is_cell_hazardous(from_coord)

	var step_dir = Vector3i(
		roundi(float(diff.x) / steps),
		roundi(float(diff.y) / steps),
		roundi(float(diff.z) / steps)
	)

	for i in range(steps):
		var next_cell = current + step_dir
		if is_cell_hazardous(next_cell):
			return true
		current = next_cell

	return false
