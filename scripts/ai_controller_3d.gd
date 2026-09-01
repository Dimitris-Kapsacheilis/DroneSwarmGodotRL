extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")
@onready var obstacle_spawner = get_node_or_null("/root/Swarm Test/Obstacles")
@onready var nfz_manager = get_node_or_null("/root/Swarm Test/NoFlyZoneManager") # Adjust path if needed

@export var enable_action_masking: bool = false

@export var max_tracked_frontiers: int = 3
@export var max_tracked_nfz: int = 3
@export var max_tracked_obstacles: int = 3

@export var max_velocity_reference: float = 5.0
@export var max_distance_reference: float = 30.0

# FIX: these are now actually used in get_reward() instead of hardcoded
# literals. Values updated to match what the reward function had been using
# (15 / 20 / 15 / 50 / 0.1) so nothing changes behaviorally — but now tuning
# these in the Inspector actually has an effect.
@export var completion_bonus: float = 50.0
@export var nfz_violation_penalty: float = 15.0
@export var obstacle_collision_penalty: float = 20.0
@export var battery_depletion_penalty: float = 15.0
@export var voxel_reward_weight: float = 0.1
@export var time_step_penalty: float = 0.01
@export var invalid_move_penalty: float = 1.0

@export var frontier_shaping_weight: float = 1.0
@export var obstacle_shaping_weight: float = 0.5
@export var obstacle_danger_radius: float = 5.0

var cached_centroids: Array = []
var _cached_nfz_nodes: Array = []
var actions_taken := 0
var previous_coverage := 0.0
var previous_discovered_voxels := 0
var coverage_threshold = 80
var violated := false
var hit_obstacle := false
var battery_depleted := false
var _signals_connected := false

var _prev_frontier_dist: float = -1.0
var _prev_nearest_obstacle_dist: float = -1.0
var _current_frontier_dist: float = -1.0
var _current_nearest_obstacle_dist: float = -1.0

# 6 Cardinal Directions for adjacent Octree Groups
const DIRECTIONS = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1)
]

func _ready() -> void:
	super._ready()
	_connect_drone_signals()

func _physics_process(_delta: float) -> void:
	if done or violated or hit_obstacle or battery_depleted:
		return
	if not _signals_connected:
		_connect_drone_signals()

	if is_instance_valid(navigator) and is_instance_valid(navigator.drone):
		var drone_pos = navigator.drone.global_position

		if "current_battery" in navigator.drone and navigator.drone.current_battery <= 0.0:
			battery_depleted = true
			return

		if _is_inside_any_nfz(drone_pos):
			violated = true
#
			#print("--- NFZ VIOLATION REGISTERED ---")
			#print("Drone Global Position: ", drone_pos)
			#print("Drone Grid Coordinate: ", _get_current_grid_pos())
			#print("Target Waypoint: ", navigator.current_target_pos)
			#print("Path Size Remaining: ", navigator.current_path.size())
			#if navigator.current_path.size() > 0:
				#print("Active Trajectory Queue: ", navigator.current_path)
#
			#for zone in _get_nfz_nodes():
				#if zone.has_method("contains_position") and zone.contains_position(drone_pos):
					#print("Violated Node ID: ", zone.name)
					#print("Boundary Altitude: [", zone.min_altitude, " - ", zone.max_altitude, "]")
					#print("Polygon Points: ", zone.polygon)
			#print("---------------------------------")
			return

		_check_imminent_obstacle_danger(drone_pos)

# Uses the enforcement-gated is_straight_path_safe() on purpose: with
# enforce_blocked_cells = false this is a no-op (never cancels a flight),
# which is intentional — the agent is meant to be free to fly into a hazard
# and experience the consequence, not have the environment steer it away
# mid-flight. This only becomes active if enforce_blocked_cells is ever
# turned back on, or once obstacles are made to move again (it exists for
# re-checking a path against obstacles that moved after the decision).
func _check_imminent_obstacle_danger(drone_pos: Vector3) -> void:
	if not is_instance_valid(grid_manager) or not grid_manager.has_method("is_straight_path_safe"):
		return
	if not navigator.has_target:
		return

	var current_grid_pos = _get_current_grid_pos()
	var target_grid_pos = grid_manager.world_to_grid(navigator.current_target_pos) if grid_manager.has_method("world_to_grid") else current_grid_pos

	if not grid_manager.is_straight_path_safe(current_grid_pos, target_grid_pos):
		if navigator.has_method("cancel_current_target"):
			navigator.cancel_current_target()

func _connect_drone_signals() -> void:
	if _signals_connected:
		return
	if is_instance_valid(navigator) and is_instance_valid(navigator.drone):
		var drone_node = navigator.drone
		if drone_node.has_signal("collided"):
			if not drone_node.collided.is_connected(_on_drone_collided):
				drone_node.collided.connect(_on_drone_collided)
			_signals_connected = true

func _on_drone_collided(collider: Node) -> void:
	if not violated and not hit_obstacle and not battery_depleted:
		hit_obstacle = true

func _get_current_grid_pos() -> Vector3i:
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
		return Vector3i.ZERO

	var drone_pos = navigator.drone.global_position
	if grid_manager.has_method("world_to_grid"):
		return grid_manager.world_to_grid(drone_pos)
	return Vector3i(floor(drone_pos.x), floor(drone_pos.y), floor(drone_pos.z))

# =====================================================
# OBSERVATIONS & ACTION MASKING
# =====================================================

func get_obs() -> Dictionary:
	var total_obs_size = 8 + (max_tracked_frontiers * 3) + (max_tracked_nfz * 3) + (max_tracked_obstacles * 6) + 6

	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
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

	var pos = navigator.drone.global_position
	var vel = navigator.drone.linear_velocity if navigator.drone is RigidBody3D else navigator.velocity
	var grid_limits = Vector3(grid_manager.grid_size)

	var norm_pos_x = clampf(pos.x / max(grid_limits.x, 0.001), 0.0, 1.0)
	var norm_pos_y = clampf(pos.y / max(grid_limits.y, 0.001), 0.0, 1.0)
	var norm_pos_z = clampf(pos.z / max(grid_limits.z, 0.001), 0.0, 1.0)

	var norm_vel_x = clampf(vel.x / max_velocity_reference, -1.0, 1.0)
	var norm_vel_y = clampf(vel.y / max_velocity_reference, -1.0, 1.0)
	var norm_vel_z = clampf(vel.z / max_velocity_reference, -1.0, 1.0)

	var coverage = grid_manager.get_coverage_percentage() / 100.0

	var battery_ratio = 1.0
	if navigator.drone.has_method("get_battery_ratio"):
		battery_ratio = navigator.drone.get_battery_ratio()
	elif "current_battery" in navigator.drone and "max_battery" in navigator.drone:
		battery_ratio = clampf(navigator.drone.current_battery / maxf(navigator.drone.max_battery, 1.0), 0.0, 1.0)

	var obs: Array = [
		norm_pos_x, norm_pos_y, norm_pos_z,
		norm_vel_x, norm_vel_y, norm_vel_z,
		coverage,
		battery_ratio
	]

	# 1. Frontier vectors
	var frontier_obs: Array[float] = []
	var centroids = grid_manager.get_frontier_centroids(max_tracked_frontiers) if (not navigator.has_target or cached_centroids.is_empty()) else cached_centroids
	cached_centroids = centroids
	centroids.sort_custom(func(a, b): return pos.distance_squared_to(a) < pos.distance_squared_to(b))

	for i in range(max_tracked_frontiers):
		if i < centroids.size():
			var rel_vector = centroids[i] - pos
			frontier_obs.append(clampf(rel_vector.x / max_distance_reference, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.y / max_distance_reference, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.z / max_distance_reference, -1.0, 1.0))
		else:
			frontier_obs.append(1.0); frontier_obs.append(1.0); frontier_obs.append(1.0)
	obs.append_array(frontier_obs)

	# 2. NFZ vectors
	var nfz_relative_vectors: Array[float] = []
	var nfz_nodes = _get_nfz_nodes()
	var distance_mappings = []
	for zone in nfz_nodes:
		var rel_vec = _get_relative_vector_to_nfz_aabb(pos, zone)
		distance_mappings.append({"distance": rel_vec.length(), "rel_vector": rel_vec})
	distance_mappings.sort_custom(func(a, b): return a.distance < b.distance)

	for i in range(max_tracked_nfz):
		if i < distance_mappings.size():
			var rel_vec = distance_mappings[i].rel_vector
			nfz_relative_vectors.append(clampf(rel_vec.x / max_distance_reference, -1.0, 1.0))
			nfz_relative_vectors.append(clampf(rel_vec.y / max_distance_reference, -1.0, 1.0))
			nfz_relative_vectors.append(clampf(rel_vec.z / max_distance_reference, -1.0, 1.0))
		else:
			nfz_relative_vectors.append(1.0); nfz_relative_vectors.append(1.0); nfz_relative_vectors.append(1.0)
	obs.append_array(nfz_relative_vectors)

	# 3. Obstacle vectors
	var obstacle_obs: Array[float] = []
	var all_obstacles = get_tree().get_nodes_in_group("obstacles")
	all_obstacles.sort_custom(func(a, b): return pos.distance_squared_to(a.global_position) < pos.distance_squared_to(b.global_position))

	var nearest_obstacle_dist: float = INF
	for i in range(max_tracked_obstacles):
		if i < all_obstacles.size():
			var obs_node = all_obstacles[i] as Node3D
			var rel_pos = obs_node.global_position - pos
			if i == 0: nearest_obstacle_dist = rel_pos.length()

			var obs_vel = obs_node.linear_velocity if obs_node is RigidBody3D else (obs_node.velocity if "velocity" in obs_node else Vector3.ZERO)
			var rel_vel = obs_vel - vel

			obstacle_obs.append(clampf(rel_pos.x / max_distance_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_pos.y / max_distance_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_pos.z / max_distance_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_vel.x / max_velocity_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_vel.y / max_velocity_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_vel.z / max_velocity_reference, -1.0, 1.0))
		else:
			obstacle_obs.append(1.0); obstacle_obs.append(1.0); obstacle_obs.append(1.0)
			obstacle_obs.append(0.0); obstacle_obs.append(0.0); obstacle_obs.append(0.0)
	obs.append_array(obstacle_obs)

	# 4. 6-Directional Clearance Observation
	# FIX: this now uses is_straight_path_hazardous(), which always reflects
	# real obstacle/NFZ occupancy regardless of enforce_blocked_cells. The
	# previous version used is_straight_path_safe(), whose obstacle-awareness
	# is gated by enforce_blocked_cells — with that flag false (intentionally,
	# so the agent can actually experience collisions and learn from them),
	# this observation would otherwise always report every direction as
	# "safe" even standing right next to an obstacle, leaving the agent with
	# no usable signal at all. set_action() below is untouched and still uses
	# the enforcement-gated check, so the agent remains free to actually fly
	# into a hazard it was warned about.
	var current_grid_pos = _get_current_grid_pos()
	var directional_clearance: Array[float] = []
	var action_mask: Array[float] = []

	for dir_idx in range(DIRECTIONS.size()):
		var target_coord = grid_manager.get_adjacent_octree_center(current_grid_pos, dir_idx)
		var is_hazard = true

		if target_coord != current_grid_pos and grid_manager.is_within_grid_limits(target_coord):
			if grid_manager.has_method("is_straight_path_hazardous"):
				is_hazard = grid_manager.is_straight_path_hazardous(current_grid_pos, target_coord)
			else:
				is_hazard = false

		directional_clearance.append(-1.0 if is_hazard else 1.0)
		action_mask.append(0.0 if is_hazard else 1.0)

	obs.append_array(directional_clearance)

	# Update potential tracking (done at decision points only, matching the
	# cadence get_reward() applies shaping at)
	if not navigator.has_target:
		_current_nearest_obstacle_dist = nearest_obstacle_dist if nearest_obstacle_dist != INF else max_distance_reference
		if _prev_nearest_obstacle_dist < 0.0:
			_prev_nearest_obstacle_dist = _current_nearest_obstacle_dist

		if centroids.size() > 0:
			var closest_d = INF
			for c in centroids: closest_d = minf(closest_d, pos.distance_to(c))
			_current_frontier_dist = closest_d
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

	# Terminal failures
	if violated:
		reward = -nfz_violation_penalty
		done = true; needs_reset = true
		return reward

	if hit_obstacle:
		reward = -obstacle_collision_penalty
		done = true; needs_reset = true
		return reward

	if battery_depleted:
		reward = -battery_depletion_penalty
		done = true; needs_reset = true
		return reward

	# Penalty for trying to fly into wall/obstacle when unmasked
	if _invalid_move_penalized:
		_invalid_move_penalized = false
		reward = -invalid_move_penalty
		return reward

	if navigator.has_target:
		return 0.0

	# Progress reward
	var coverage = grid_manager.get_coverage_percentage()
	var current_voxels = grid_manager.visited_cells.size()
	var new_voxels = current_voxels - previous_discovered_voxels

	reward = float(new_voxels) * voxel_reward_weight

	# Small time step penalty so drone doesn't loiter
	reward -= time_step_penalty

	# ---- Potential-based frontier shaping ----
	# Rewards genuine progress toward the nearest frontier (distance decreased
	# since the last decision point), not raw proximity, so it can't be
	# farmed by sitting near a frontier without exploring it.
	if _prev_frontier_dist >= 0.0 and _current_frontier_dist >= 0.0:
		reward += (_prev_frontier_dist - _current_frontier_dist) * frontier_shaping_weight
	_prev_frontier_dist = _current_frontier_dist

	# ---- Potential-based obstacle-avoidance shaping ----
	# FIX: this block was missing entirely in the previous version even though
	# obstacle_shaping_weight/obstacle_danger_radius and the distance trackers
	# were still declared. Without it there was zero continuous training
	# signal for obstacle avoidance beyond the sparse terminal collision
	# penalty. Symmetric to the frontier term above: reward opening distance
	# to the nearest obstacle, penalize closing it, only while within
	# obstacle_danger_radius so it doesn't interfere with normal exploration
	# far from any obstacle.
	if _prev_nearest_obstacle_dist >= 0.0 and _current_nearest_obstacle_dist >= 0.0:
		if _current_nearest_obstacle_dist < obstacle_danger_radius or _prev_nearest_obstacle_dist < obstacle_danger_radius:
			var obstacle_progress = _current_nearest_obstacle_dist - _prev_nearest_obstacle_dist
			reward += obstacle_progress * obstacle_shaping_weight
	_prev_nearest_obstacle_dist = _current_nearest_obstacle_dist

	if coverage >= coverage_threshold:
		reward += completion_bonus
		print("EPISODE STEPS : ", n_steps, " , ACTIONS TAKEN : ", actions_taken, " , PERCENTAGE : %.2f%%" % coverage, " DONE!!!!! [SUCCESS]")
		done = true; needs_reset = true

	previous_coverage = coverage
	previous_discovered_voxels = current_voxels
	return reward

func get_done() -> bool:
	if not is_instance_valid(grid_manager):
		return false

	var coverage = grid_manager.get_coverage_percentage()

	if coverage >= coverage_threshold:
		done = true
		needs_reset = true
		return true

	if violated:
		print("EPISODE STEPS : ", n_steps, " , ACTIONS TAKEN : ", actions_taken, " , PERCENTAGE : %.2f%%" % coverage, " [FAILED: NFZ VIOLATION]")
		done = true
		needs_reset = true
		return true

	if hit_obstacle:
		print("EPISODE STEPS : ", n_steps, " , ACTIONS TAKEN : ", actions_taken, " , PERCENTAGE : %.2f%%" % coverage, " [FAILED: OBSTACLE COLLISION]")
		done = true
		needs_reset = true
		return true

	if battery_depleted:
		print("EPISODE STEPS : ", n_steps, " , ACTIONS TAKEN : ", actions_taken, " , PERCENTAGE : %.2f%%" % coverage, " [FAILED: BATTERY DEPLETED]")
		done = true
		needs_reset = true
		return true

	return false

# =====================================================
# ACTION SPACE & HANDLING
# =====================================================

func get_action_space() -> Dictionary:
	return {
		"flight_waypoint": {
			"action_type": "discrete",
			"size": DIRECTIONS.size()
		}
	}

var _invalid_move_penalized := false

func set_action(action) -> void:
	if done or not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
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
	var target_coord = grid_manager.get_adjacent_octree_center(current_grid_pos, action_idx)

	var is_safe = grid_manager.is_within_bounds(target_coord) and grid_manager.is_straight_path_safe(current_grid_pos, target_coord)

	if not is_safe:
		_invalid_move_penalized = true
		target_coord = current_grid_pos

	navigator.perform_action(target_coord)

# =====================================================
# RESET & UTILITIES
# =====================================================
func reset() -> void:
	super.reset()
	needs_reset = false
	done = false
	violated = false
	hit_obstacle = false
	battery_depleted = false
	actions_taken = 0
	previous_coverage = 0.0
	previous_discovered_voxels = 0
	n_steps = 0
	_cached_nfz_nodes.clear()
	_signals_connected = false
	_prev_frontier_dist = -1.0
	_prev_nearest_obstacle_dist = -1.0
	_current_frontier_dist = -1.0
	_current_nearest_obstacle_dist = -1.0

	# 1. Reset and respawn random NFZs first
	if is_instance_valid(nfz_manager):
		if nfz_manager.has_method("reset_nfz"):
			nfz_manager.reset_nfz()
		elif nfz_manager.has_method("generate_zones"):
			nfz_manager.generate_zones()

	# 2. Rebuild the grid BEFORE obstacles re-register.
	# FIX: this used to run AFTER obstacle_spawner.reset_obstacles(). Since
	# grid_manager.reset_grid() unconditionally clears obstacle_cells, running
	# it after obstacles registered themselves wiped that registration out for
	# the rest of the episode — obstacles would never be reflected in grid
	# safety checks or the directional-clearance observation. Grid reset must
	# happen first so obstacle registrations (via register_obstacle(), called
	# from obstacle_spawner.reset_obstacles()) land afterward and stick.
	if is_instance_valid(grid_manager):
		grid_manager.reset_grid()

	# 3. Reset dynamic obstacles (re-registers them into the now-clean grid)
	if is_instance_valid(obstacle_spawner) and obstacle_spawner.has_method("reset_obstacles"):
		obstacle_spawner.reset_obstacles()

	# 4. Spawn/Position the Drone in a verified clear cell
	if is_instance_valid(swarm_controller):
		swarm_controller.reset_swarm_pos()

	if is_instance_valid(navigator):
		navigator.reset_rl_stats()
		if is_instance_valid(navigator.drone) and navigator.drone.has_method("reset_battery"):
			navigator.drone.reset_battery()

	_connect_drone_signals()

func set_done_false():
	done = false

func _get_nfz_nodes() -> Array:
	if not _cached_nfz_nodes.is_empty():
		return _cached_nfz_nodes
	var found: Array = []
	var root = get_tree().current_scene
	if root:
		_get_nfz_nodes_recursive(root, found)
	_cached_nfz_nodes = found
	return found

func _get_nfz_nodes_recursive(node: Node, found: Array) -> void:
	if node is NoFlyZone:
		found.append(node)
	for child in node.get_children():
		_get_nfz_nodes_recursive(child, found)

func _get_relative_vector_to_nfz_aabb(pos: Vector3, zone: Node) -> Vector3:
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return Vector3(999.0, 999.0, 999.0)
	var points_array = zone.polygon
	if points_array.size() == 0:
		return Vector3(999.0, 999.0, 999.0)

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

	var closest_x = clampf(pos.x, min_x, max_x)
	var closest_y = clampf(pos.y, zone.min_altitude, zone.max_altitude)
	var closest_z = clampf(pos.z, min_z, max_z)
	return Vector3(closest_x, closest_y, closest_z) - pos

func _get_distance_to_nfz_aabb(pos: Vector3, zone: Node) -> float:
	return _get_relative_vector_to_nfz_aabb(pos, zone).length()

func _is_inside_any_nfz(pos: Vector3) -> bool:
	for zone in _get_nfz_nodes():
		if zone.has_method("contains_position") and zone.contains_position(pos):
			return true
	return false
