extends Node3D

const Drone = preload("res://scripts/drone.gd")
var rng = RandomNumberGenerator.new()

@export var drone_packed_scene: PackedScene
@export var num_drones: int = 2
@export var is_rl_training: bool = true
var current_leader: Drone = null
@export var grid_manager: Node3D
@export var nfz_manager: Node3D
@export var obstacle_spawner: Node3D

@export var random_spawn: bool = true
@export var default_spawn_position: Vector3 = Vector3(1.5, 1.5, 1.5)

var drones: Array[Drone] = []
var _last_reset_frame: int = -1
var swarm_failed: bool = false

var _drone_colors := [
	Color(1.0, 0.2, 0.2),
	Color(0.2, 1.0, 0.2),
	Color(0.2, 0.6, 1.0),
	Color(1.0, 1.0, 0.2),
]

func _ready() -> void:
	rng.randomize()

	if not drone_packed_scene:
		push_error("Assign your drone.tscn!")
		return

	for i in range(num_drones):
		var drone: Drone = drone_packed_scene.instantiate()
		drone.name = "Drone_" + str(i)
		drone.drone_id = i
		drone.drone_color = _drone_colors[i % _drone_colors.size()]
		drone.swarm_controller = self
		drone.add_to_group("drones")

		add_child(drone)
		drones.append(drone)
		if drones.size() > 0:
			current_leader = drones[0]
	var sync_node = get_tree().current_scene.get_node_or_null("Sync")
	if is_instance_valid(sync_node) and "agents" in sync_node:
		sync_node.agents = get_tree().get_nodes_in_group("AGENT")

	reset_environment()

# Inside swarm_controller.gd

func trigger_swarm_failure(reason: String = "", agent_actions: int = -1) -> void:
	if not swarm_failed:
		var coverage_val := 0.0
		if is_instance_valid(grid_manager) and grid_manager.has_method("get_coverage_percentage"):
			coverage_val = grid_manager.get_coverage_percentage()

		# Compute total actions across all drones
		var total_actions := 0
		if agent_actions >= 0:
			total_actions = agent_actions
		else:
			for d in drones:
				if is_instance_valid(d):
					var nav = d.get_node_or_null("Navigator")
					if is_instance_valid(nav) and "actions_taken" in nav:
						total_actions += nav.actions_taken

		print("[SWARM TERMINATION] -> Reason: %s | Coverage: %.2f%% | Actions Taken: %d" % [reason, coverage_val, total_actions])
		swarm_failed = true

func reset_environment() -> void:
	var current_frame = Engine.get_physics_frames()
	if _last_reset_frame == current_frame:
		return
	_last_reset_frame = current_frame
	swarm_failed = false

	# 1. Reset NFZs
	if not is_instance_valid(nfz_manager):
		nfz_manager = get_node_or_null("../NoFlyZoneManager")
	if is_instance_valid(nfz_manager) and nfz_manager.has_method("reset_nfz"):
		nfz_manager.reset_nfz()

	# 2. Reset Grid
	if not is_instance_valid(grid_manager):
		grid_manager = get_node_or_null("../GridManager")
	if is_instance_valid(grid_manager) and grid_manager.has_method("reset_grid"):
		grid_manager.reset_grid()

	# 3. Reset Obstacles
	if not is_instance_valid(obstacle_spawner):
		obstacle_spawner = get_node_or_null("../Obstacles")
	if is_instance_valid(obstacle_spawner) and obstacle_spawner.has_method("reset_obstacles"):
		obstacle_spawner.reset_obstacles()

	# 4. Respawn drones
	reset_swarm_pos()

func reset_swarm_pos() -> void:
	var spawn_positions = _find_clustered_free_cells(drones.size())

	for i in range(drones.size()):
		var drone = drones[i]
		drone.global_position = spawn_positions[i]
		drone.rotation = Vector3.ZERO
		drone.transform.basis = Basis.IDENTITY

		if drone is RigidBody3D:
			drone.linear_velocity = Vector3.ZERO
			drone.angular_velocity = Vector3.ZERO
		elif "velocity" in drone:
			drone.velocity = Vector3.ZERO

		var nav = drone.get_node_or_null("Navigator")
		if is_instance_valid(nav):
			if nav.has_method("cancel_current_target"):
				nav.cancel_current_target()
			if nav.has_method("reset_rl_stats"):
				nav.reset_rl_stats()

		if drone.has_method("clear_targets"):
			drone.clear_targets()

func _find_clustered_free_cells(count: int) -> Array[Vector3]:
	var result: Array[Vector3] = []
	if not is_instance_valid(grid_manager):
		for i in range(count):
			result.append(default_spawn_position + Vector3(i * 1.5, 0, 0))
		return result

	var grid_size: Vector3i = grid_manager.grid_size
	var occupied_cells: Dictionary = {}

	var seed_coord = _get_random_free_cell_coord(grid_size, occupied_cells)

	var neighbor_offsets: Array[Vector3i] = []
	for dx in range(-1, 2):
		for dy in range(-1, 2):
			for dz in range(-1, 2):
				if dx != 0 or dy != 0 or dz != 0:
					neighbor_offsets.append(Vector3i(dx, dy, dz))
	neighbor_offsets.sort_custom(func(a: Vector3i, b: Vector3i) -> bool: return a.length_squared() < b.length_squared())

	var queue: Array[Vector3i] = [seed_coord]
	var visited: Dictionary = {seed_coord: true}

	while queue.size() > 0 and result.size() < count:
		var current = queue.pop_front()

		if _is_cell_free(current, grid_size, occupied_cells):
			occupied_cells[current] = true
			result.append(grid_manager.grid_to_world(current))

		for offset in neighbor_offsets:
			var neighbor = current + offset
			if not visited.has(neighbor) and _is_in_bounds(neighbor, grid_size):
				visited[neighbor] = true
				queue.append(neighbor)

	while result.size() < count:
		var fallback = _get_random_free_cell_coord(grid_size, occupied_cells)
		occupied_cells[fallback] = true
		result.append(grid_manager.grid_to_world(fallback))

	return result

func _get_random_free_cell_coord(grid_size: Vector3i, occupied_cells: Dictionary) -> Vector3i:
	for _attempt in range(300):
		var rx = rng.randi_range(1, grid_size.x - 2)
		var ry = rng.randi_range(1, grid_size.y - 2)
		var rz = rng.randi_range(1, grid_size.z - 2)
		var coord = Vector3i(rx, ry, rz)
		if _is_cell_free(coord, grid_size, occupied_cells):
			return coord
	return Vector3i(1, 1, 1)

func _is_cell_free(coord: Vector3i, grid_size: Vector3i, occupied_cells: Dictionary) -> bool:
	if not _is_in_bounds(coord, grid_size) or occupied_cells.has(coord):
		return false
	if is_instance_valid(grid_manager) and grid_manager.has_method("is_cell_strictly_free"):
		return grid_manager.is_cell_strictly_free(coord)
	return true

func _is_in_bounds(coord: Vector3i, grid_size: Vector3i) -> bool:
	return coord.x >= 0 and coord.x < grid_size.x and coord.y >= 0 and coord.y < grid_size.y and coord.z >= 0 and coord.z < grid_size.z

func _physics_process(_delta: float) -> void:
	if is_rl_training:
		return
