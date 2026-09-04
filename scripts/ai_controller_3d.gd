extends AIController3D

const Drone = preload("res://scripts/drone.gd")

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var swarm_controller = get_node_or_null("/root/Swarm Test/Swarm Controller")
@onready var obstacle_spawner = get_node_or_null("/root/Swarm Test/Obstacles")
@onready var nfz_manager = get_node_or_null("/root/Swarm Test/NoFlyZoneManager")

var drone: Node3D = null
var navigator: Node3D = null

@export_group("Safety Enforcement")
## When FALSE: The drone is allowed to fly into hazards and crash to learn negative penalties.
## When TRUE: Hazardous actions are blocked before execution.
@export var enforce_blocked_cells: bool = false
@export var enable_action_masking: bool = false

@export_group("Observation Dimensions")
@export var max_tracked_frontiers: int = 3
@export var max_tracked_nfz: int = 3
@export var max_tracked_obstacles: int = 3
@export var max_tracked_teammates: int = 3

@export var max_velocity_reference: float = 5.0
@export var max_distance_reference: float = 30.0

@export_group("Rewards & Penalties")
@export var completion_bonus: float = 50.0
@export var nfz_violation_penalty: float = 15.0
@export var obstacle_collision_penalty: float = 20.0
@export var teammate_collision_penalty: float = 20.0
@export var battery_depletion_penalty: float = 15.0
@export var voxel_reward_weight: float = 0.1
@export var time_step_penalty: float = 0.01
@export var invalid_move_penalty: float = 1.0

@export_group("Potential Shaping")
@export var frontier_shaping_weight: float = 1.0
@export var obstacle_shaping_weight: float = 0.5
@export var obstacle_danger_radius: float = 5.0

var cached_centroids: Array = []
var _cached_nfz_nodes: Array = []
var actions_taken := 0
var previous_coverage := 0.0
var previous_discovered_voxels := 0
var coverage_threshold = 95.0

var violated := false
var hit_obstacle := false
var hit_teammate := false
var battery_depleted := false
var _signals_connected := false

var _prev_frontier_dist: float = -1.0
var _prev_nearest_obstacle_dist: float = -1.0
var _current_frontier_dist: float = -1.0
var _current_nearest_obstacle_dist: float = -1.0
var _invalid_move_penalized := false

const DIRECTIONS = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1)
]

func _ready() -> void:
	super._ready()
	_resolve_drone_and_navigator()
	_connect_drone_signals()

func _resolve_drone_and_navigator() -> void:
	var parent = get_parent()
	if not is_instance_valid(parent):
		return

	if parent is Drone or parent.has_signal("collided") or "velocity" in parent:
		drone = parent
		navigator = parent.get_node_or_null("Navigator")
	elif parent.name == "Navigator" or "drone" in parent:
		navigator = parent
		drone = parent.drone if ("drone" in parent and is_instance_valid(parent.drone)) else parent.get_parent()
	else:
		drone = parent
		navigator = parent.get_node_or_null("Navigator")

func _physics_process(_delta: float) -> void:
	if done or violated or hit_obstacle or hit_teammate or battery_depleted:
		return
	if not _signals_connected or not is_instance_valid(drone):
		_resolve_drone_and_navigator()
		_connect_drone_signals()

	if is_instance_valid(drone):
		var drone_pos = drone.global_position

		if "current_battery" in drone and drone.current_battery <= 0.0:
			battery_depleted = true
			if is_instance_valid(swarm_controller):
				swarm_controller.trigger_swarm_failure("Battery Depleted (" + drone.name + ")", actions_taken)
			return

		if _is_inside_any_nfz(drone_pos):
			violated = true
			if is_instance_valid(swarm_controller):
				swarm_controller.trigger_swarm_failure("NFZ Violation (" + drone.name + ")", actions_taken)
			return

		_check_imminent_obstacle_danger(drone_pos)

func _check_imminent_obstacle_danger(_drone_pos: Vector3) -> void:
	if not enforce_blocked_cells:
		return

	if not is_instance_valid(grid_manager) or not grid_manager.has_method("is_straight_path_safe"):
		return
	if not is_instance_valid(navigator) or not navigator.has_target:
		return

	var current_grid_pos = _get_current_grid_pos()
	var target_grid_pos = grid_manager.world_to_grid(navigator.current_target_pos) if grid_manager.has_method("world_to_grid") else current_grid_pos

	if not grid_manager.is_straight_path_safe(current_grid_pos, target_grid_pos):
		if navigator.has_method("cancel_current_target"):
			navigator.cancel_current_target()

func _connect_drone_signals() -> void:
	if _signals_connected or not is_instance_valid(drone):
		return
	if drone.has_signal("collided") and not drone.collided.is_connected(_on_drone_collided):
		drone.collided.connect(_on_drone_collided)
		_signals_connected = true

func _on_drone_collided(collider: Node) -> void:
	if done or violated or hit_obstacle or hit_teammate or battery_depleted:
		return

	if is_instance_valid(collider) and (collider.is_in_group("drones") or collider is Drone):
		hit_teammate = true
		if is_instance_valid(swarm_controller):
			swarm_controller.trigger_swarm_failure("Teammate Collision (" + drone.name + ")", actions_taken)
	else:
		hit_obstacle = true
		if is_instance_valid(swarm_controller):
			swarm_controller.trigger_swarm_failure("Obstacle Collision (" + drone.name + ")", actions_taken)

func _get_current_grid_pos() -> Vector3i:
	if not is_instance_valid(drone) or not is_instance_valid(grid_manager):
		return Vector3i.ZERO
	if grid_manager.has_method("world_to_grid"):
		return grid_manager.world_to_grid(drone.global_position)
	return Vector3i(int(floor(drone.global_position.x)), int(floor(drone.global_position.y)), int(floor(drone.global_position.z)))

# =====================================================
# OBSERVATIONS
# =====================================================

func get_obs() -> Dictionary:
	var total_obs_size = 8 + (max_tracked_frontiers * 3) + (max_tracked_nfz * 3) + (max_tracked_obstacles * 6) + (max_tracked_teammates * 6) + 6

	if not is_instance_valid(drone) or not is_instance_valid(grid_manager):
		var empty_obs: Array = []
		empty_obs.resize(total_obs_size)
		empty_obs.fill(0.0)
		var return_dict: Dictionary = { "obs": empty_obs }
		if enable_action_masking:
			var empty_mask: Array[float] = []
			empty_mask.resize(DIRECTIONS.size())
			empty_mask.fill(0.0)
			return_dict["action_mask"] = empty_mask
		return return_dict

	var pos = drone.global_position
	var vel = drone.linear_velocity if drone is RigidBody3D else (drone.velocity if "velocity" in drone else Vector3.ZERO)
	var grid_limits = Vector3(grid_manager.grid_size) if "grid_size" in grid_manager else Vector3(30, 30, 30)

	var norm_pos = Vector3(
		clampf(pos.x / max(grid_limits.x, 0.001), 0.0, 1.0),
		clampf(pos.y / max(grid_limits.y, 0.001), 0.0, 1.0),
		clampf(pos.z / max(grid_limits.z, 0.001), 0.0, 1.0)
	)
	var norm_vel = Vector3(
		clampf(vel.x / max_velocity_reference, -1.0, 1.0),
		clampf(vel.y / max_velocity_reference, -1.0, 1.0),
		clampf(vel.z / max_velocity_reference, -1.0, 1.0)
	)

	var coverage = (grid_manager.get_coverage_percentage() / 100.0) if grid_manager.has_method("get_coverage_percentage") else 0.0
	var battery_ratio = 1.0
	if drone.has_method("get_battery_ratio"):
		battery_ratio = drone.get_battery_ratio()
	elif "current_battery" in drone and "max_battery" in drone:
		battery_ratio = clampf(drone.current_battery / maxf(drone.max_battery, 1.0), 0.0, 1.0)

	var obs: Array = [
		norm_pos.x, norm_pos.y, norm_pos.z,
		norm_vel.x, norm_vel.y, norm_vel.z,
		coverage,
		battery_ratio
	]

	# 1. Frontiers
	var centroids = grid_manager.get_frontier_centroids(max_tracked_frontiers) if (grid_manager.has_method("get_frontier_centroids") and (not is_instance_valid(navigator) or not navigator.has_target or cached_centroids.is_empty())) else cached_centroids
	cached_centroids = centroids
	centroids.sort_custom(func(a, b): return pos.distance_squared_to(a) < pos.distance_squared_to(b))

	for i in range(max_tracked_frontiers):
		if i < centroids.size():
			var rel = (centroids[i] - pos) / max_distance_reference
			obs.append_array([clampf(rel.x, -1.0, 1.0), clampf(rel.y, -1.0, 1.0), clampf(rel.z, -1.0, 1.0)])
		else:
			obs.append_array([1.0, 1.0, 1.0])

	# 2. NFZs
	var nfz_nodes = _get_nfz_nodes()
	var distance_mappings = []
	for zone in nfz_nodes:
		if is_instance_valid(zone) and not zone.is_queued_for_deletion():
			var rel_vec = _get_relative_vector_to_nfz_aabb(pos, zone)
			distance_mappings.append({"distance": rel_vec.length(), "rel_vector": rel_vec})
	distance_mappings.sort_custom(func(a, b): return a.distance < b.distance)

	for i in range(max_tracked_nfz):
		if i < distance_mappings.size():
			var rel = distance_mappings[i].rel_vector / max_distance_reference
			obs.append_array([clampf(rel.x, -1.0, 1.0), clampf(rel.y, -1.0, 1.0), clampf(rel.z, -1.0, 1.0)])
		else:
			obs.append_array([1.0, 1.0, 1.0])

	# 3. Obstacles
	var raw_obstacles = get_tree().get_nodes_in_group("obstacles") if get_tree() else []
	var obstacles = raw_obstacles.filter(func(n): return is_instance_valid(n) and not n.is_queued_for_deletion())
	obstacles.sort_custom(func(a, b): return pos.distance_squared_to(a.global_position) < pos.distance_squared_to(b.global_position))

	var nearest_obstacle_dist: float = INF
	for i in range(max_tracked_obstacles):
		if i < obstacles.size():
			var obs_node = obstacles[i] as Node3D
			var rel_pos = (obs_node.global_position - pos) / max_distance_reference
			if i == 0: nearest_obstacle_dist = (obs_node.global_position - pos).length()
			var obs_vel = obs_node.linear_velocity if obs_node is RigidBody3D else (obs_node.velocity if "velocity" in obs_node else Vector3.ZERO)
			var rel_vel = (obs_vel - vel) / max_velocity_reference
			obs.append_array([
				clampf(rel_pos.x, -1.0, 1.0), clampf(rel_pos.y, -1.0, 1.0), clampf(rel_pos.z, -1.0, 1.0),
				clampf(rel_vel.x, -1.0, 1.0), clampf(rel_vel.y, -1.0, 1.0), clampf(rel_vel.z, -1.0, 1.0)
			])
		else:
			obs.append_array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

	# 4. Teammates
	var raw_drones = get_tree().get_nodes_in_group("drones") if get_tree() else []
	var other_drones = raw_drones.filter(func(d): return is_instance_valid(d) and d != drone and not d.is_queued_for_deletion())
	other_drones.sort_custom(func(a, b): return pos.distance_squared_to(a.global_position) < pos.distance_squared_to(b.global_position))

	for i in range(max_tracked_teammates):
		if i < other_drones.size():
			var team = other_drones[i] as Node3D
			var rel_pos = (team.global_position - pos) / max_distance_reference
			var team_vel = team.linear_velocity if team is RigidBody3D else (team.velocity if "velocity" in team else Vector3.ZERO)
			var rel_vel = (team_vel - vel) / max_velocity_reference
			obs.append_array([
				clampf(rel_pos.x, -1.0, 1.0), clampf(rel_pos.y, -1.0, 1.0), clampf(rel_pos.z, -1.0, 1.0),
				clampf(rel_vel.x, -1.0, 1.0), clampf(rel_vel.y, -1.0, 1.0), clampf(rel_vel.z, -1.0, 1.0)
			])
		else:
			obs.append_array([1.0, 1.0, 1.0, 0.0, 0.0, 0.0])

	# 5. Clearance
	var current_grid_pos = _get_current_grid_pos()
	var action_mask: Array[float] = []

	for dir_idx in range(DIRECTIONS.size()):
		var target_coord = grid_manager.get_adjacent_octree_center(current_grid_pos, dir_idx) if grid_manager.has_method("get_adjacent_octree_center") else current_grid_pos
		var is_hazard = true
		if target_coord != current_grid_pos and grid_manager.has_method("is_within_grid_limits") and grid_manager.is_within_grid_limits(target_coord):
			if grid_manager.has_method("is_straight_path_hazardous"):
				is_hazard = grid_manager.is_straight_path_hazardous(current_grid_pos, target_coord)
			else:
				is_hazard = false
		obs.append(-1.0 if is_hazard else 1.0)
		action_mask.append(0.0 if is_hazard else 1.0)

	if is_instance_valid(navigator) and not navigator.has_target:
		_current_nearest_obstacle_dist = nearest_obstacle_dist if nearest_obstacle_dist != INF else max_distance_reference
		if _prev_nearest_obstacle_dist < 0.0:
			_prev_nearest_obstacle_dist = _current_nearest_obstacle_dist
		if centroids.size() > 0:
			_current_frontier_dist = pos.distance_to(centroids[0])
			if _prev_frontier_dist < 0.0:
				_prev_frontier_dist = _current_frontier_dist

	var result: Dictionary = { "obs": obs }
	if enable_action_masking:
		result["action_mask"] = action_mask
	return result

# =====================================================
# REWARD & TERMINATION
# =====================================================

func get_reward() -> float:
	if not is_instance_valid(grid_manager):
		return 0.0

	if violated:
		reward = -nfz_violation_penalty
		done = true; needs_reset = true
		return reward

	if hit_obstacle:
		reward = -obstacle_collision_penalty
		done = true; needs_reset = true
		return reward

	if hit_teammate:
		reward = -teammate_collision_penalty
		done = true; needs_reset = true
		return reward

	if battery_depleted:
		reward = -battery_depletion_penalty
		done = true; needs_reset = true
		return reward

	if _invalid_move_penalized:
		_invalid_move_penalized = false
		reward = -invalid_move_penalty
		return reward

	if is_instance_valid(navigator) and navigator.has_target:
		return 0.0

	var coverage = grid_manager.get_coverage_percentage() if grid_manager.has_method("get_coverage_percentage") else 0.0
	var current_voxels = grid_manager.visited_cells.size() if "visited_cells" in grid_manager else 0
	var new_voxels = max(0, current_voxels - previous_discovered_voxels)

	reward = float(new_voxels) * voxel_reward_weight
	reward -= time_step_penalty

	if _prev_frontier_dist >= 0.0 and _current_frontier_dist >= 0.0:
		reward += (_prev_frontier_dist - _current_frontier_dist) * frontier_shaping_weight
	_prev_frontier_dist = _current_frontier_dist

	if _prev_nearest_obstacle_dist >= 0.0 and _current_nearest_obstacle_dist >= 0.0:
		if _current_nearest_obstacle_dist < obstacle_danger_radius or _prev_nearest_obstacle_dist < obstacle_danger_radius:
			var obstacle_progress = _current_nearest_obstacle_dist - _prev_nearest_obstacle_dist
			reward += obstacle_progress * obstacle_shaping_weight
	_prev_nearest_obstacle_dist = _current_nearest_obstacle_dist

	if coverage >= coverage_threshold:
		reward += completion_bonus
		done = true; needs_reset = true
		if is_instance_valid(swarm_controller):
			swarm_controller.trigger_swarm_failure("Coverage Success [SUCCESS] !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", actions_taken)

	previous_discovered_voxels = current_voxels
	return reward

func get_done() -> bool:
	if needs_reset:
		return true

	if is_instance_valid(swarm_controller) and swarm_controller.swarm_failed:
		done = true
		needs_reset = true
		return true

	if not is_instance_valid(grid_manager):
		return false
	if grid_manager.has_method("get_coverage_percentage") and grid_manager.get_coverage_percentage() >= coverage_threshold:
		done = true; needs_reset = true
		return true
	if violated or hit_obstacle or hit_teammate or battery_depleted:
		done = true; needs_reset = true
		return true
	return done

# =====================================================
# ACTION SPACE
# =====================================================

func get_action_space() -> Dictionary:
	return {
		"flight_waypoint": {
			"action_type": "discrete",
			"size": DIRECTIONS.size()
		}
	}

func set_action(action) -> void:
	if done or not is_instance_valid(navigator) or not is_instance_valid(drone) or not is_instance_valid(grid_manager):
		return

	if navigator.has_target:
		actions_taken += 1
		return

	var action_idx = 0
	if action is Dictionary and action.has("flight_waypoint"):
		var val = action["flight_waypoint"]
		action_idx = int(val[0]) if (val is Array or val is PackedFloat32Array or val is PackedInt32Array) else int(val)
	elif action is Array or action is PackedFloat32Array or action is PackedInt32Array:
		action_idx = int(action[0])
	else:
		action_idx = int(action)

	action_idx = clampi(action_idx, 0, DIRECTIONS.size() - 1)
	actions_taken += 1

	var current_grid_pos = _get_current_grid_pos()
	var target_coord = grid_manager.get_adjacent_octree_center(current_grid_pos, action_idx) if grid_manager.has_method("get_adjacent_octree_center") else current_grid_pos
	
	target_coord.x = clampi(target_coord.x, 0, grid_manager.grid_size.x - 1)
	target_coord.y = clampi(target_coord.y, 0, grid_manager.grid_size.y - 1)
	target_coord.z = clampi(target_coord.z, 0, grid_manager.grid_size.z - 1)

	if enforce_blocked_cells:
		var is_safe = grid_manager.has_method("is_straight_path_safe") and grid_manager.is_straight_path_safe(current_grid_pos, target_coord)
		if not is_safe:
			_invalid_move_penalized = true
			target_coord = current_grid_pos

	if navigator.has_method("perform_action"):
		navigator.perform_action(target_coord)

# =====================================================
# RESET
# =====================================================

func reset() -> void:
	super.reset()
	needs_reset = false
	done = false
	violated = false
	hit_obstacle = false
	hit_teammate = false
	battery_depleted = false
	_invalid_move_penalized = false

	actions_taken = 0
	previous_discovered_voxels = 0
	n_steps = 0
	_cached_nfz_nodes.clear()
	_signals_connected = false
	_prev_frontier_dist = -1.0
	_current_frontier_dist = -1.0
	_prev_nearest_obstacle_dist = -1.0
	_current_nearest_obstacle_dist = -1.0

	if is_instance_valid(swarm_controller) and swarm_controller.has_method("reset_environment"):
		swarm_controller.reset_environment()

	if not is_instance_valid(drone) or not is_instance_valid(navigator):
		_resolve_drone_and_navigator()

	if is_instance_valid(navigator) and navigator.has_method("reset_rl_stats"):
		navigator.reset_rl_stats()
	if is_instance_valid(drone) and drone.has_method("reset_battery"):
		drone.reset_battery()

	_connect_drone_signals()

func set_done_false():
	done = false

# =====================================================
# NFZ UTILITIES
# =====================================================

func _get_nfz_nodes() -> Array:
	_cached_nfz_nodes = _cached_nfz_nodes.filter(func(n): return is_instance_valid(n) and not n.is_queued_for_deletion())
	if not _cached_nfz_nodes.is_empty():
		return _cached_nfz_nodes

	var found: Array = []
	var root = get_tree().current_scene if get_tree() else null
	if root:
		_get_nfz_nodes_recursive(root, found)
	_cached_nfz_nodes = found
	return _cached_nfz_nodes

func _get_nfz_nodes_recursive(node: Node, found: Array) -> void:
	if not is_instance_valid(node) or node.is_queued_for_deletion():
		return
	if node is NoFlyZone:
		found.append(node)
	for child in node.get_children():
		_get_nfz_nodes_recursive(child, found)

func _is_inside_any_nfz(pos: Vector3) -> bool:
	var zones = _get_nfz_nodes()
	for zone in zones:
		if not is_instance_valid(zone) or zone.is_queued_for_deletion():
			continue

		if zone.has_method("contains_position"):
			if zone.contains_position(pos):
				return true

		if "min_altitude" in zone and "max_altitude" in zone and "polygon" in zone:
			if pos.y >= zone.min_altitude and pos.y <= zone.max_altitude:
				if Geometry2D.is_point_in_polygon(Vector2(pos.x, pos.z), zone.polygon):
					return true
	return false

func _get_relative_vector_to_nfz_aabb(pos: Vector3, zone: Node) -> Vector3:
	if not is_instance_valid(zone) or zone.is_queued_for_deletion():
		return Vector3(999.0, 999.0, 999.0)
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return Vector3(999.0, 999.0, 999.0)

	var points_array = zone.polygon
	if points_array.size() == 0:
		return Vector3(999.0, 999.0, 999.0)

	var min_x = points_array[0].x; var max_x = points_array[0].x
	var min_z = points_array[0].y; var max_z = points_array[0].y
	for i in range(1, points_array.size()):
		var p = points_array[i]
		if p.x < min_x: min_x = p.x
		elif p.x > max_x: max_x = p.x
		if p.y < min_z: min_z = p.y
		elif p.y > max_z: max_z = p.y

	var closest_x = clampf(pos.x, min_x, max_x)
	var closest_y = clampf(pos.y, zone.min_altitude, zone.max_altitude)
	var closest_z = clampf(pos.z, min_z, max_z)
	return Vector3(closest_x, closest_y, closest_z) - pos
