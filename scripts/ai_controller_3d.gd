extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")
@onready var obstacle_spawner = get_node_or_null("/root/Swarm Test/Obstacles")
@export var max_tracked_frontiers: int = 3
@export var max_tracked_nfz: int = 3
@export var max_tracked_obstacles: int = 3 # K-nearest obstacles to track

# ---------------------------------------------------------------------------
# Scaling constants for normalization. ALL observations are normalized to
# roughly [-1, 1] (or [0, 1] for coverage/battery). This matters a lot for PPO:
# unnormalized position (0-30), velocity (-5..5), and relative vectors of
# wildly different magnitudes will destabilize training and make it very
# hard for the network to learn anything meaningful, including obstacle
# avoidance.
# ---------------------------------------------------------------------------
@export var max_velocity_reference: float = 5.0     # Expected max speed of the drone
@export var max_distance_reference: float = 30.0    # Distance ceiling for NFZ/obstacle/frontier observations
@export var completion_bonus: float = 2000.0
@export var nfz_violation_penalty: float = 500.0
@export var obstacle_collision_penalty: float = 1000.0 # Penalty for hitting an obstacle
@export var battery_depletion_penalty: float = 1000.0  # Penalty for running out of battery

# Reward shaping weights. Keep these small relative to the terminal rewards
# above, and note both shaping terms are POTENTIAL-BASED (reward = change in
# distance, not raw proximity). A raw-proximity term pays the agent just for
# sitting near a frontier/obstacle forever, which competes with actually
# covering the grid. A potential-based term only pays for genuine progress
# and nets to (near) zero over a full episode.
@export var frontier_shaping_weight: float = 1.0
@export var obstacle_shaping_weight: float = 0.5
@export var obstacle_danger_radius: float = 5.0   # Only shape obstacle avoidance within this range
@export var voxel_reward_weight: float = 0.01

var cached_centroids: Array = []
var _cached_nfz_nodes: Array = []
var actions_taken := 0
var previous_coverage := 0.0
var previous_discovered_voxels := 0
var coverage_threshold = 80
var violated := false
var hit_obstacle := false # Tracks if the drone hit an obstacle during the step
var battery_depleted := false # Tracks if the drone battery is fully discharged
var _signals_connected := false # Track signal status

# Potential-based shaping trackers (distance at the last decision point)
var _prev_frontier_dist: float = -1.0
var _prev_nearest_obstacle_dist: float = -1.0

# Discrete action space variables
const DIRECTIONS = [
	Vector3i(1, 0, 0), Vector3i(-1, 0, 0),
	Vector3i(0, 1, 0), Vector3i(0, -1, 0),
	Vector3i(0, 0, 1), Vector3i(0, 0, -1)
]
const OFFSETS = [1]#, 5, 10]

func _ready() -> void:
	super._ready()
	_connect_drone_signals()

func _process(_delta: float) -> void:
	return

# Monitors position on every physics frame to catch high-speed violations,
# and re-checks whether the in-flight path is still safe as obstacles move.
func _physics_process(_delta: float) -> void:
	if done or violated or hit_obstacle or battery_depleted:
		return
	# Safe fall-back: try to connect signals if they weren't ready at startup
	if not _signals_connected:
		_connect_drone_signals()

	if is_instance_valid(navigator) and is_instance_valid(navigator.drone):
		var drone_pos = navigator.drone.global_position
		
		# Check for battery exhaustion
		if "current_battery" in navigator.drone and navigator.drone.current_battery <= 0.0:
			battery_depleted = true
			return

		if _is_inside_any_nfz(drone_pos):
			violated = true

			# Diagnostic log layout to identify flight edge cases
			print("--- NFZ VIOLATION REGISTERED ---")
			print("Drone Global Position: ", drone_pos)
			print("Drone Grid Coordinate: ", _get_current_grid_pos())
			print("Target Waypoint: ", navigator.current_target_pos)
			print("Path Size Remaining: ", navigator.current_path.size())
			if navigator.current_path.size() > 0:
				print("Active Trajectory Queue: ", navigator.current_path)

			for zone in _get_nfz_nodes():
				if zone.has_method("contains_position") and zone.contains_position(drone_pos):
					print("Violated Node ID: ", zone.name)
					print("Boundary Altitude: [", zone.min_altitude, " - ", zone.max_altitude, "]")
					print("Polygon Points: ", zone.polygon)
			print("---------------------------------")
			return

		# Reactive obstacle check: since flight to a waypoint is open-loop,
		# a moving obstacle can wander into the path *after* the decision was
		# made. Without this check the agent is sometimes punished for
		# collisions it had no way to avoid at decision time, which badly
		# corrupts the credit assignment for obstacle-avoidance learning.
		_check_imminent_obstacle_danger(drone_pos)

func _check_imminent_obstacle_danger(drone_pos: Vector3) -> void:
	if not is_instance_valid(grid_manager) or not grid_manager.has_method("is_straight_path_safe"):
		return
	if not navigator.has_target:
		return

	var current_grid_pos = _get_current_grid_pos()
	var target_grid_pos = grid_manager.world_to_grid(navigator.current_target_pos) if grid_manager.has_method("world_to_grid") else current_grid_pos

	if not grid_manager.is_straight_path_safe(current_grid_pos, target_grid_pos):
		# The remaining path is no longer safe (an obstacle moved into it).
		# Ask the navigator to abandon the current waypoint so a fresh
		# decision (with up-to-date obstacle info) is requested instead of
		# flying blindly into danger.
		if navigator.has_method("cancel_current_target"):
			navigator.cancel_current_target()

# Establishes connections with the main drone signals
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

# Helper to find the current grid cell coordinate of the drone
func _get_current_grid_pos() -> Vector3i:
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
		return Vector3i.ZERO

	var drone_pos = navigator.drone.global_position
	if grid_manager.has_method("world_to_grid"):
		return grid_manager.world_to_grid(drone_pos)
	elif grid_manager.has_method("global_to_map"):
		return grid_manager.global_to_map(drone_pos)
	elif grid_manager.has_method("local_to_map"):
		return grid_manager.local_to_map(grid_manager.to_local(drone_pos))
	else:
		return Vector3i(
			floor(drone_pos.x),
			floor(drone_pos.y),
			floor(drone_pos.z)
		)

# =====================================================
# OBSERVATIONS & ACTION MASKING
# =====================================================

func get_obs() -> Dictionary:
	var action_mask: Array[float] = []
	action_mask.resize(DIRECTIONS.size() * OFFSETS.size())
	action_mask.fill(0.0)

	# The base observation vector contains 8 values:
	# norm_pos (3), norm_vel (3), coverage (1), battery_ratio (1)
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
		var empty_obs: Array = []
		empty_obs.resize(8 + max_tracked_frontiers * 3 + max_tracked_nfz * 3 + max_tracked_obstacles * 6)
		empty_obs.fill(0.0)
		return {
			"obs": empty_obs,
			"action_mask": action_mask
		}

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
	
	# Read and normalize current battery level
	var battery_ratio = 1.0
	if navigator.drone.has_method("get_battery_ratio"):
		battery_ratio = navigator.drone.get_battery_ratio()
	elif "current_battery" in navigator.drone and "max_battery" in navigator.drone:
		battery_ratio = clampf(navigator.drone.current_battery / maxf(navigator.drone.max_battery, 1.0), 0.0, 1.0)
	print(battery_ratio)
	var obs: Array = [
		norm_pos_x, norm_pos_y, norm_pos_z,
		norm_vel_x, norm_vel_y, norm_vel_z,
		coverage,
		battery_ratio
	]

	# ---------------- Frontier Observation Parsing ----------------
	var frontier_obs: Array[float] = []
	var centroids: Array = []

	if not navigator.has_target or cached_centroids.is_empty():
		centroids = grid_manager.get_frontier_centroids(max_tracked_frontiers)
		cached_centroids = centroids
	else:
		centroids = cached_centroids

	centroids.sort_custom(func(a, b):
		return pos.distance_squared_to(a) < pos.distance_squared_to(b)
	)

	for i in range(max_tracked_frontiers):
		if i < centroids.size():
			var rel_vector = centroids[i] - pos
			frontier_obs.append(clampf(rel_vector.x / max_distance_reference, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.y / max_distance_reference, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.z / max_distance_reference, -1.0, 1.0))
		else:
			frontier_obs.append(1.0)
			frontier_obs.append(1.0)
			frontier_obs.append(1.0)

	obs.append_array(frontier_obs)

	# ---------------- NFZ Observation Parsing ----------------
	var nfz_relative_vectors: Array[float] = []
	var nfz_nodes = _get_nfz_nodes()

	var distance_mappings = []
	for zone in nfz_nodes:
		var rel_vec = _get_relative_vector_to_nfz_aabb(pos, zone)
		var dist = rel_vec.length()
		distance_mappings.append({"zone": zone, "distance": dist, "rel_vector": rel_vec})

	distance_mappings.sort_custom(func(a, b):
		return a.distance < b.distance
	)

	for i in range(max_tracked_nfz):
		if i < distance_mappings.size():
			var rel_vec = distance_mappings[i].rel_vector
			nfz_relative_vectors.append(clampf(rel_vec.x / max_distance_reference, -1.0, 1.0))
			nfz_relative_vectors.append(clampf(rel_vec.y / max_distance_reference, -1.0, 1.0))
			nfz_relative_vectors.append(clampf(rel_vec.z / max_distance_reference, -1.0, 1.0))
		else:
			nfz_relative_vectors.append(1.0)
			nfz_relative_vectors.append(1.0)
			nfz_relative_vectors.append(1.0)

	obs.append_array(nfz_relative_vectors)

	# ---------------- Obstacle Observation Parsing (K-Nearest) ----------------
	var obstacle_obs: Array[float] = []
	var all_obstacles = get_tree().get_nodes_in_group("obstacles")

	all_obstacles.sort_custom(func(a, b):
		return pos.distance_squared_to(a.global_position) < pos.distance_squared_to(b.global_position)
	)

	var nearest_obstacle_dist: float = INF

	for i in range(max_tracked_obstacles):
		if i < all_obstacles.size():
			var obs_node = all_obstacles[i] as Node3D
			var rel_pos = obs_node.global_position - pos

			if i == 0:
				nearest_obstacle_dist = rel_pos.length()

			var obs_velocity = Vector3.ZERO
			if obs_node is RigidBody3D:
				obs_velocity = obs_node.linear_velocity
			elif "velocity" in obs_node:
				obs_velocity = obs_node.velocity

			var rel_vel = obs_velocity - vel

			# Relative Position (normalized)
			obstacle_obs.append(clampf(rel_pos.x / max_distance_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_pos.y / max_distance_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_pos.z / max_distance_reference, -1.0, 1.0))

			# Relative Velocity (normalized)
			obstacle_obs.append(clampf(rel_vel.x / max_velocity_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_vel.y / max_velocity_reference, -1.0, 1.0))
			obstacle_obs.append(clampf(rel_vel.z / max_velocity_reference, -1.0, 1.0))
		else:
			obstacle_obs.append(1.0)
			obstacle_obs.append(1.0)
			obstacle_obs.append(1.0)

			obstacle_obs.append(0.0)
			obstacle_obs.append(0.0)
			obstacle_obs.append(0.0)

	obs.append_array(obstacle_obs)

	# Track nearest obstacle distance for potential-based shaping in get_reward()
	if nearest_obstacle_dist == INF:
		nearest_obstacle_dist = max_distance_reference

	if not navigator.has_target:
		# Only update the "previous" distance at decision points, matching
		# the cadence at which get_reward() actually applies shaping.
		if _prev_nearest_obstacle_dist < 0.0:
			_prev_nearest_obstacle_dist = nearest_obstacle_dist
		_current_nearest_obstacle_dist = nearest_obstacle_dist

		if centroids.size() > 0:
			var closest_frontier_dist = INF
			for c in centroids:
				var d = pos.distance_to(c)
				if d < closest_frontier_dist:
					closest_frontier_dist = d
			if _prev_frontier_dist < 0.0:
				_prev_frontier_dist = closest_frontier_dist
			_current_frontier_dist = closest_frontier_dist

	# ---------------- Action Masking ----------------
	action_mask.clear()
	var current_grid_pos = _get_current_grid_pos()

	for offset_idx in range(len(OFFSETS)):
		var step = OFFSETS[offset_idx]
		for dir_idx in range(DIRECTIONS.size()):
			var dir = DIRECTIONS[dir_idx]
			var target_coord = current_grid_pos + (dir * step)

			if grid_manager.has_method("is_straight_path_safe") and grid_manager.is_straight_path_safe(current_grid_pos, target_coord):
				action_mask.append(1.0)
			elif grid_manager.is_within_bounds(target_coord) and not grid_manager.has_method("is_straight_path_safe"):
				action_mask.append(1.0)
			else:
				action_mask.append(0.0)

	return {
		"obs": obs,
		"action_mask": action_mask
	}

# Scratch values computed in get_obs() at decision points, consumed by
# get_reward() immediately after. Declared here so both functions can see
# them without recomputing frontier/obstacle distances twice per step.
var _current_frontier_dist: float = -1.0
var _current_nearest_obstacle_dist: float = -1.0

# =====================================================
# REWARD
# =====================================================
func get_reward() -> float:
	if not is_instance_valid(grid_manager):
		return 0.0

	# 1. Handle terminal failures immediately, even if en route
	if violated:
		reward = -nfz_violation_penalty
		done = true
		needs_reset = true
		return reward

	if hit_obstacle:
		reward = -obstacle_collision_penalty
		done = true
		needs_reset = true
		return reward

	if battery_depleted:
		reward = -battery_depletion_penalty
		done = true
		needs_reset = true
		return reward

	# 2. Return 0.0 while in transit to prevent stale reward accumulation
	if navigator.has_target:
		return 0.0

	# 3. Calculate rewards once a waypoint is reached/decision point is active
	var coverage = grid_manager.get_coverage_percentage()
	var current_voxels = 0

	if grid_manager.has_method("get_explored_count"):
		current_voxels = grid_manager.get_explored_count()
	elif grid_manager.has_method("get_discovered_count"):
		current_voxels = grid_manager.get_discovered_count()
	elif "grid_size" in grid_manager:
		var total_cells = grid_manager.grid_size.x * grid_manager.grid_size.y * grid_manager.grid_size.z
		current_voxels = floor((coverage / 100.0) * total_cells)

	var new_voxels_discovered = current_voxels - previous_discovered_voxels
	reward = float(new_voxels_discovered) * voxel_reward_weight

	# ---- Potential-based frontier shaping ----
	if _prev_frontier_dist >= 0.0 and _current_frontier_dist >= 0.0:
		var frontier_progress = _prev_frontier_dist - _current_frontier_dist
		reward += frontier_progress * frontier_shaping_weight
	_prev_frontier_dist = _current_frontier_dist

	# ---- Potential-based obstacle-avoidance shaping ----
	if _prev_nearest_obstacle_dist >= 0.0 and _current_nearest_obstacle_dist >= 0.0:
		if _current_nearest_obstacle_dist < obstacle_danger_radius or _prev_nearest_obstacle_dist < obstacle_danger_radius:
			var obstacle_progress = _current_nearest_obstacle_dist - _prev_nearest_obstacle_dist
			reward += obstacle_progress * obstacle_shaping_weight
	_prev_nearest_obstacle_dist = _current_nearest_obstacle_dist

	# 4. Handle success/completion
	if coverage >= coverage_threshold:
		reward += completion_bonus
		print("EPISODE STEPS : ", n_steps, " , ACTIONS TAKEN : ", actions_taken, " DONE!!!!! ")
		done = true
		needs_reset = true

	# Update tracking variables
	previous_coverage = coverage
	previous_discovered_voxels = current_voxels

	return reward

# =====================================================
# TERMINATION
# =====================================================

func get_done() -> bool:
	if not is_instance_valid(grid_manager):
		return false

	var coverage = grid_manager.get_coverage_percentage()

	if coverage >= coverage_threshold:
		done = true
		needs_reset = true
		return true

	if violated:
		print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken, " , PERCENTAGE : ", coverage, " [FAILED: NFZ VIOLATION]")
		done = true
		needs_reset = true
		return true

	if hit_obstacle:
		print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken, " , PERCENTAGE : ", coverage, " [FAILED: OBSTACLE COLLISION]")
		done = true
		needs_reset = true
		return true

	if battery_depleted:
		print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken, " , PERCENTAGE : ", coverage, " [FAILED: BATTERY DEPLETED]")
		done = true
		needs_reset = true
		return true

	return false

# =====================================================
# ACTION SPACE
# =====================================================

func get_action_space() -> Dictionary:
	return {
		"flight_waypoint": {
			"action_type": "discrete",
			"size": DIRECTIONS.size() * OFFSETS.size()
		}
	}

# =====================================================
# ACTION HANDLING
# =====================================================

func set_action(action) -> void:
	if done:
		return

	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone):
		return

	if not is_instance_valid(grid_manager):
		return

	if navigator.has_target:
		actions_taken += 1
		return

	var action_idx: int = 0

	if action is Dictionary:
		if not action.has("flight_waypoint"):
			return
		var val = action["flight_waypoint"]
		if val is Array or val is PackedFloat32Array or val is PackedInt32Array:
			action_idx = int(val[0])
		else:
			action_idx = int(val)
	elif action is Array or action is PackedFloat32Array or action is PackedInt32Array:
		action_idx = int(action[0])
	else:
		action_idx = int(action)

	action_idx = clampi(action_idx, 0, DIRECTIONS.size() * OFFSETS.size() - 1)
	actions_taken += 1

	var current_grid_pos = _get_current_grid_pos()

	var direction_idx = action_idx % DIRECTIONS.size()
	var offset_idx = action_idx / DIRECTIONS.size()

	var selected_direction = DIRECTIONS[direction_idx]
	var selected_offset = OFFSETS[offset_idx]

	var target_coord = current_grid_pos + (selected_direction * selected_offset)

	if not grid_manager.is_within_bounds(target_coord):
		var found_safe_alternative = false
		for alt_offset_idx in range(len(OFFSETS)):
			var alt_step = OFFSETS[alt_offset_idx]
			var alt_coord = current_grid_pos + (selected_direction * alt_step)
			if grid_manager.is_within_bounds(alt_coord):
				target_coord = alt_coord
				found_safe_alternative = true
				break

		if not found_safe_alternative:
			target_coord = current_grid_pos

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

	if is_instance_valid(navigator):
		navigator.reset_rl_stats()
		if is_instance_valid(navigator.drone) and navigator.drone.has_method("reset_battery"):
			navigator.drone.reset_battery()

	if is_instance_valid(grid_manager):
		grid_manager.reset_grid()

	if is_instance_valid(swarm_controller):
		swarm_controller.reset_swarm_pos()

	if is_instance_valid(obstacle_spawner) and obstacle_spawner.has_method("reset_obstacles"):
		obstacle_spawner.reset_obstacles()

	_connect_drone_signals()

func set_done_false():
	done = false

# =====================================================
# INTERNAL NO-FLY ZONE UTILITIES
# =====================================================

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

	var closest_point = Vector3(closest_x, closest_y, closest_z)
	return closest_point - pos

func _get_distance_to_nfz_aabb(pos: Vector3, zone: Node) -> float:
	return _get_relative_vector_to_nfz_aabb(pos, zone).length()

func _is_inside_any_nfz(pos: Vector3) -> bool:
	for zone in _get_nfz_nodes():
		if zone.has_method("contains_position") and zone.contains_position(pos):
			return true
	return false
