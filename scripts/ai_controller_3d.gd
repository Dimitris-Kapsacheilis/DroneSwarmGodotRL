extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")

@export var max_tracked_frontiers: int = 3
@export var max_action_radius: float = 10.0 # Maximum radius in grid units for the next waypoint

# Scaling constants for clean normalization
@export var max_velocity_reference: float = 5.0     # Expected max speed of the drone
@export var max_distance_reference: float = 30.0    # Distance ceiling for NFZ observations
@export var completion_bonus: float = 10.0          # Normalized from 100.0
@export var nfz_violation_penalty: float = 5.0      # Normalized from 100.0

var cached_centroids: Array = []
var _cached_nfz_nodes: Array = [] 
var actions_taken := 0
var previous_coverage := 0.0
var coverage_threshold = 95

func _ready() -> void:
	super._ready()

func _process(_delta: float) -> void:
	return

# =====================================================
# OBSERVATIONS (All Normalized to [-1, 1] or [0, 1])
# =====================================================

func get_obs() -> Dictionary:
	# Default fallback matches the 19 features dimension
	var obs = [
		0.0, 0.0, 0.0, # Position
		0.0, 0.0, 0.0, # Velocity
		0.0,           # Coverage
		0.0, 0.0, 0.0, # Frontier 1 relative
		0.0, 0.0, 0.0, # Frontier 2 relative
		0.0, 0.0, 0.0, # Frontier 3 relative
		1.0, 1.0, 1.0  # Normalized distances to NFZ (1.0 means safe/unseen)
	]
	
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
		return {"obs": obs}

	var pos = navigator.drone.global_position
	var vel = navigator.velocity
	var grid_limits = Vector3(grid_manager.grid_size)

	# 1. Normalize Position to [0, 1] based on grid size
	var norm_pos_x = clampf(pos.x / grid_limits.x, 0.0, 1.0)
	var norm_pos_y = clampf(pos.y / grid_limits.y, 0.0, 1.0)
	var norm_pos_z = clampf(pos.z / grid_limits.z, 0.0, 1.0)

	# 2. Normalize Velocity to [-1, 1] using max reference speed
	var norm_vel_x = clampf(vel.x / max_velocity_reference, -1.0, 1.0)
	var norm_vel_y = clampf(vel.y / max_velocity_reference, -1.0, 1.0)
	var norm_vel_z = clampf(vel.z / max_velocity_reference, -1.0, 1.0)

	# 3. Coverage is already naturally [0, 1]
	var coverage = grid_manager.get_coverage_percentage() / 100.0
		
	obs = [ 
		norm_pos_x, norm_pos_y, norm_pos_z,
		norm_vel_x, norm_vel_y, norm_vel_z,
		coverage
	]
	
	# 4. Normalize Frontier observations to [-1, 1] based on local search window
	var frontier_obs: Array[float] = []
	var centroids: Array = []
	
	if not navigator.has_target or cached_centroids.is_empty():
		centroids = grid_manager.get_frontier_centroids(3)
		cached_centroids = centroids 
	else:
		centroids = cached_centroids 

	centroids.sort_custom(func(a, b):
		return pos.distance_squared_to(a) < pos.distance_squared_to(b)
	)
	
	var search_radius = float(grid_manager.local_search_radius)
	for i in range(max_tracked_frontiers):
		if i < centroids.size():
			var rel_vector = centroids[i] - pos
			# Scale and clamp relative distance to standard [-1, 1]
			frontier_obs.append(clampf(rel_vector.x / search_radius, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.y / search_radius, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.z / search_radius, -1.0, 1.0))
		else:
			frontier_obs.append(0.0)
			frontier_obs.append(0.0)
			frontier_obs.append(0.0)
			
	obs.append_array(frontier_obs)
	
	# 5. Normalize NFZ distances to [0, 1]
	var nfz_distances: Array[float] = []
	var nfz_nodes = _get_nfz_nodes()
	
	var distance_mappings = []
	for zone in nfz_nodes:
		var dist = _get_distance_to_nfz_aabb(pos, zone)
		distance_mappings.append({"zone": zone, "distance": dist})
		
	distance_mappings.sort_custom(func(a, b):
		return a.distance < b.distance
	)
	
	for i in range(3):
		if i < distance_mappings.size():
			var raw_dist = distance_mappings[i].distance
			# Maps raw distance (0 to 30+) to normalized [0, 1]
			var norm_dist = clampf(raw_dist / max_distance_reference, 0.0, 1.0)
			nfz_distances.append(norm_dist)
		else:
			nfz_distances.append(1.0) # 1.0 represents the safest (infinite) distance limit

	obs.append_array(nfz_distances)
	
	return {
		"obs": obs
	}

# =====================================================
# REWARD
# =====================================================

func get_reward() -> float:
	if not is_instance_valid(grid_manager):
		return 0.0

	var coverage = grid_manager.get_coverage_percentage() 
	
	if not navigator.has_target: 
		# 1. Reward newly discovered coverage (Value scaled from 2.0 down to 1.0 for range control)
		reward = (coverage - previous_coverage) * 1.0
		
		# 2. Living penalty
		reward -= 0.001 * actions_taken
		
		# 3. Frontier Proximity Reward
		if is_instance_valid(navigator.drone):
			var pos = navigator.drone.global_position
			
			# Check for immediate NFZ violation
			if _is_inside_any_nfz(pos):
				reward -= nfz_violation_penalty
				done = true
				needs_reset = true
				return reward
			
			var centroids = grid_manager.get_frontier_centroids(3)
			if centroids.size() > 0:
				var closest_dist = INF
				for c in centroids:
					var d = pos.distance_to(c)
					if d < closest_dist:
						closest_dist = d
				
				# Normalizes shaping bonus to stay strictly between [0.0, 0.5]
				var proximity_shaping = 0.5 / (1.0 + closest_dist)
				reward += proximity_shaping
		
		# 4. Completion bonus
		if coverage >= coverage_threshold:
			reward += completion_bonus
			print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken)
			done = true
			needs_reset = true
			
		previous_coverage = coverage
		
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

	# Check for termination due to NFZ violation
	if is_instance_valid(navigator) and is_instance_valid(navigator.drone):
		if _is_inside_any_nfz(navigator.drone.global_position):
			print("AGENT PENALIZED: Entered No-Fly Zone!")
			print("Steps:",n_steps," ,percentage: " ,coverage,"%%")
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
			"action_type": "continuous",
			"size": 3
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
		print("HAS TARGET")
		actions_taken += 1
		return

	var act = []

	if action is Dictionary:
		if not action.has("flight_waypoint"):
			print("no waypoint action")
			return

		act = action["flight_waypoint"]
	else:
		act = action

	if act.size() < 3:
		print("no 3 size action")
		return
	
	actions_taken += 1

	var current_grid_pos = Vector3i.ZERO
	var drone_pos = navigator.drone.global_position

	if grid_manager.has_method("world_to_grid"):
		current_grid_pos = grid_manager.world_to_grid(drone_pos)
	elif grid_manager.has_method("global_to_map"):
		current_grid_pos = grid_manager.global_to_map(drone_pos)
	elif grid_manager.has_method("local_to_map"):
		current_grid_pos = grid_manager.local_to_map(grid_manager.to_local(drone_pos))
	else:
		current_grid_pos = Vector3i(
			roundi(drone_pos.x),
			roundi(drone_pos.y),
			roundi(drone_pos.z)
		)

	var displacement = Vector3(act[0], act[1], act[2])
	if displacement.length() > 1.0:
		displacement = displacement.normalized()

	var offset = displacement * max_action_radius
	var target_grid_pos = Vector3(current_grid_pos) + offset

	var target_coord = Vector3i(
		clampi(roundi(target_grid_pos.x), 0, grid_manager.grid_size.x - 1),
		clampi(roundi(target_grid_pos.y), 0, grid_manager.grid_size.y - 1),
		clampi(roundi(target_grid_pos.z), 0, grid_manager.grid_size.z - 1)
	)
	navigator.perform_action(target_coord)
	

# =====================================================
# RESET
# =====================================================

func reset() -> void:
	print("RESET CALLED")

	super.reset()
	needs_reset = false
	done = false
	actions_taken = 0
	previous_coverage = 0.0
	n_steps = 0
	_cached_nfz_nodes.clear()

	if is_instance_valid(navigator):
		navigator.reset_rl_stats()

	if is_instance_valid(grid_manager):
		grid_manager.reset_grid()
		
	if is_instance_valid(swarm_controller):
		swarm_controller.reset_swarm_pos()

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

func _get_distance_to_nfz_aabb(pos: Vector3, zone: Node) -> float:
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return 999.0
		
	var points_array = zone.polygon
	if points_array.size() == 0:
		return 999.0
		
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
	return pos.distance_to(closest_point)

func _is_inside_any_nfz(pos: Vector3) -> bool:
	for zone in _get_nfz_nodes():
		if zone.has_method("contains_position") and zone.contains_position(pos):
			return true
	return false
