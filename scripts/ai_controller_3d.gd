extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")

@export var max_tracked_frontiers: int = 3

# Safeguarded initialization to prevent startup crashes if GridManager is not yet ready
@onready var max_action_radius: float = (grid_manager.local_search_radius - 1) if is_instance_valid(grid_manager) and "local_search_radius" in grid_manager else 10.0

# Scaling constants for clean normalization
@export var max_velocity_reference: float = 5.0     # Expected max speed of the drone
@export var max_distance_reference: float = 30.0    # Distance ceiling for NFZ observations
@export var completion_bonus: float = 10.0          
@export var nfz_violation_penalty: float = 5.0      

var cached_centroids: Array = []
var _cached_nfz_nodes: Array = [] 
var actions_taken := 0
var previous_coverage := 0.0
var previous_discovered_voxels := 0
var coverage_threshold = 95
var violated := false

# Discrete action space variables
var directions_26: Array[Vector3i] = []
const OFFSETS = [1, 5, 10]

func _ready() -> void:
	super._ready()
	_initialize_directions()

func _process(_delta: float) -> void:
	return

# Monitors position on every physics frame to catch high-speed violations
func _physics_process(_delta: float) -> void:
	if done or violated:
		return
		
	if is_instance_valid(navigator) and is_instance_valid(navigator.drone):
		if _is_inside_any_nfz(navigator.drone.global_position):
			violated = true
			print("NFZ Violation registered!")

# Initialize the 26 adjacent 3D spatial directions
func _initialize_directions() -> void:
	directions_26.clear()
	for x in [-1, 0, 1]:
		for y in [-1, 0, 1]:
			for z in [-1, 0, 1]:
				if x == 0 and y == 0 and z == 0:
					continue
				directions_26.append(Vector3i(x, y, z))

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
			roundi(drone_pos.x),
			roundi(drone_pos.y),
			roundi(drone_pos.z)
		)

# =====================================================
# OBSERVATIONS & ACTION MASKING
# =====================================================

func get_obs() -> Dictionary:
	# Default fallback observations (25 dimensional)
	var obs = [
		0.0, 0.0, 0.0, # Position
		0.0, 0.0, 0.0, # Velocity
		0.0,           # Coverage
		0.0, 0.0, 0.0, # Frontier 1 relative
		0.0, 0.0, 0.0, # Frontier 2 relative
		0.0, 0.0, 0.0, # Frontier 3 relative
		0.0, 0.0, 0.0, # NFZ 1 relative vector (x, y, z)
		0.0, 0.0, 0.0, # NFZ 2 relative vector (x, y, z)
		0.0, 0.0, 0.0  # NFZ 3 relative vector (x, y, z)
	]
	
	# Fallback mask (all actions blocked if nodes are invalid)
	var action_mask: Array[float] = []
	action_mask.resize(78)
	action_mask.fill(0.0)
	
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone) or not is_instance_valid(grid_manager):
		return {
			"obs": obs,
			"action_mask": action_mask
		}

	var pos = navigator.drone.global_position
	var vel = navigator.velocity
	var grid_limits = Vector3(grid_manager.grid_size)

	# Normalize Position to [0, 1] based on grid size
	var norm_pos_x = clampf(pos.x / grid_limits.x, 0.0, 1.0)
	var norm_pos_y = clampf(pos.y / grid_limits.y, 0.0, 1.0)
	var norm_pos_z = clampf(pos.z / grid_limits.z, 0.0, 1.0)

	# Normalize Velocity to [-1, 1] using max reference speed
	var norm_vel_x = clampf(vel.x / max_velocity_reference, -1.0, 1.0)
	var norm_vel_y = clampf(vel.y / max_velocity_reference, -1.0, 1.0)
	var norm_vel_z = clampf(vel.z / max_velocity_reference, -1.0, 1.0)

	# Coverage is naturally in [0, 1]
	var coverage = grid_manager.get_coverage_percentage() / 100.0
		
	obs = [ 
		norm_pos_x, norm_pos_y, norm_pos_z,
		norm_vel_x, norm_vel_y, norm_vel_z,
		coverage
	]
	
	# Normalize Frontier observations to [-1, 1] based on local search window
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
			frontier_obs.append(clampf(rel_vector.x / search_radius, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.y / search_radius, -1.0, 1.0))
			frontier_obs.append(clampf(rel_vector.z / search_radius, -1.0, 1.0))
		else:
			frontier_obs.append(0.0)
			frontier_obs.append(0.0)
			frontier_obs.append(0.0)
			
	obs.append_array(frontier_obs)
	
	# Calculate NFZ relative vectors
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
	
	for i in range(3):
		if i < distance_mappings.size():
			var rel_vec = distance_mappings[i].rel_vector
			var norm_x = clampf(rel_vec.x / max_distance_reference, -1.0, 1.0)
			var norm_y = clampf(rel_vec.y / max_distance_reference, -1.0, 1.0)
			var norm_z = clampf(rel_vec.z / max_distance_reference, -1.0, 1.0)
			nfz_relative_vectors.append(norm_x)
			nfz_relative_vectors.append(norm_y)
			nfz_relative_vectors.append(norm_z)
		else:
			# Neutral default vectors if less than 3 zones are available
			nfz_relative_vectors.append(1.0)
			nfz_relative_vectors.append(1.0)
			nfz_relative_vectors.append(1.0)

	obs.append_array(nfz_relative_vectors)
	
	# Compute action masks for the 78 discrete actions
	action_mask.clear()
	var current_grid_pos = _get_current_grid_pos()
	
	for offset_idx in range(3):
		var step = OFFSETS[offset_idx]
		for dir_idx in range(26):
			var dir = directions_26[dir_idx]
			var target_coord = current_grid_pos + (dir * step)
			
			# SAFETY UPDATE: Check if the continuous step is clean and does not clip any corners
			if grid_manager.has_method("is_straight_path_safe") and grid_manager.is_straight_path_safe(current_grid_pos, target_coord):
				action_mask.append(1.0)
			elif grid_manager.is_within_bounds(target_coord) and not grid_manager.has_method("is_straight_path_safe"):
				# Fallback if method is missing
				action_mask.append(1.0)
			else:
				action_mask.append(0.0)
	
	return {
		"obs": obs,
		"action_mask": action_mask
	}

# =====================================================
# REWARD
# =====================================================

func get_reward() -> float:
	if not is_instance_valid(grid_manager):
		return 0.0

	var coverage = grid_manager.get_coverage_percentage() 
	
	if violated:
		reward = -nfz_violation_penalty
		done = true
		needs_reset = true
		return reward
	
	# Determine current absolute voxel count
	var current_voxels = 0
	if grid_manager.has_method("get_explored_count"):
		current_voxels = grid_manager.get_explored_count()
	elif grid_manager.has_method("get_discovered_count"):
		current_voxels = grid_manager.get_discovered_count()
	elif "grid_size" in grid_manager:
		var total_cells = grid_manager.grid_size.x * grid_manager.grid_size.y * grid_manager.grid_size.z
		current_voxels = roundi((coverage / 100.0) * total_cells)

	if not navigator.has_target: 
		# 1. Reward newly discovered coverage based on voxel count
		var new_voxels_discovered = current_voxels - previous_discovered_voxels
		reward = float(new_voxels_discovered) * 1.0
		
		# 2. Living penalty
		reward -= 0.001 * actions_taken
		
		# 3. Frontier Proximity Reward
		if is_instance_valid(navigator.drone):
			var pos = navigator.drone.global_position
			var centroids = grid_manager.get_frontier_centroids(3)
			if centroids.size() > 0:
				var closest_dist = INF
				for c in centroids:
					var d = pos.distance_to(c)
					if d < closest_dist:
						closest_dist = d
				
				var proximity_shaping = 0.5 / (1.0 + closest_dist)
				reward += proximity_shaping
		
		# 4. Completion bonus
		if coverage >= coverage_threshold:
			reward += completion_bonus
			print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken)
			done = true
			needs_reset = true
			
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
		print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken, " , PERCENTAGE : ", coverage)
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
			"size": 78
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
		print("HAS TARGET!")
		actions_taken += 1
		return

	var action_idx: int = 0

	# Safely extract discrete index
	if action is Dictionary:
		if not action.has("flight_waypoint"):
			print("no waypoint action")
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

	action_idx = clampi(action_idx, 0, 77)
	actions_taken += 1

	var current_grid_pos = _get_current_grid_pos()

	# Decode discrete action index to direction and offset magnitude
	var direction_idx = action_idx % 26
	var offset_idx = action_idx / 26

	var selected_direction = directions_26[direction_idx]
	var selected_offset = OFFSETS[offset_idx]

	var target_coord = current_grid_pos + (selected_direction * selected_offset)

	# Verify safety constraints (action masking should prevent violations, but fallbacks prevent crashes)
	if not grid_manager.is_within_bounds(target_coord):
		# Fallback: Search alternative moves starting from closest offsets if blocked
		var found_safe_alternative = false
		for alt_offset_idx in range(3):
			var alt_step = OFFSETS[alt_offset_idx]
			var alt_coord = current_grid_pos + (selected_direction * alt_step)
			if grid_manager.is_within_bounds(alt_coord):
				target_coord = alt_coord
				found_safe_alternative = true
				break
		
		# Ultimate fallback: keep drone in place
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
	actions_taken = 0
	previous_coverage = 0.0
	previous_discovered_voxels = 0
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
