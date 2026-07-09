extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")

@export var max_tracked_frontiers: int = 3
@export var max_action_radius: float = 10.0 # Maximum radius in grid units for the next waypoint
var cached_centroids: Array = []
var actions_taken := 0
var previous_coverage := 0.0
var coverage_threshold = 95

func _ready() -> void:
	super._ready()

func _process(_delta: float) -> void:
	return

# =====================================================
# OBSERVATIONS
# =====================================================

func get_obs() -> Dictionary:
	# Default observation fallback (7 basic features + 3 * 3 frontier relative coords = 16 features total)
	var obs = [
		0.0, 0.0, 0.0, # Position
		0.0, 0.0, 0.0, # Velocity
		0.0,           # Coverage
		0.0, 0.0, 0.0, # Frontier 1 relative pos
		0.0, 0.0, 0.0, # Frontier 2 relative pos
		0.0, 0.0, 0.0  # Frontier 3 relative pos
	]
	
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone):
		return {"obs": obs}

	var pos = navigator.drone.global_position
	var vel = navigator.velocity

	var coverage := 0.0
	if is_instance_valid(grid_manager):
		coverage = grid_manager.get_coverage_percentage() / 100.0
		
	# Base observations
	obs = [ 
		pos.x, pos.y, pos.z,
		vel.x, vel.y, vel.z,
		coverage
	]
	
	# Extract frontier observations using WFD
	var frontier_obs: Array[float] = []
	if is_instance_valid(grid_manager):
		var centroids: Array = []
		
		# ONLY run WFD if the drone has no target (needs a new decision) 
		# or if we have no cached data yet.
		if not navigator.has_target or cached_centroids.is_empty():
			centroids = grid_manager.get_frontier_centroids(3)
			cached_centroids = centroids # Update the cache
		else:
			centroids = cached_centroids # Use the cached data

		# Sort frontiers by distance to the drone (ascending)
		centroids.sort_custom(func(a, b):
			return pos.distance_squared_to(a) < pos.distance_squared_to(b)
		)
		
		# Append the relative vector to the top closest frontiers
		for i in range(max_tracked_frontiers):
			if i < centroids.size():
				var rel_vector = centroids[i] - pos
				frontier_obs.append(rel_vector.x)
				frontier_obs.append(rel_vector.y)
				frontier_obs.append(rel_vector.z)
			else:
				frontier_obs.append(0.0)
				frontier_obs.append(0.0)
				frontier_obs.append(0.0)
	else:
		for i in range(max_tracked_frontiers * 3):
			frontier_obs.append(0.0)
			
	obs.append_array(frontier_obs)
	#print(frontier_obs)
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
		# 1. Reward newly discovered coverage
		reward = (coverage - previous_coverage) * 2.0
		
		# 2. Living penalty
		reward -= 0.001 * actions_taken
		
		# 3. Frontier Proximity Reward
		# Adds a small shaping reward if the drone chose a waypoint close to a frontier
		if is_instance_valid(navigator.drone):
			var pos = navigator.drone.global_position
			var centroids = grid_manager.get_frontier_centroids(3)
			if centroids.size() > 0:
				var closest_dist = INF
				for c in centroids:
					var d = pos.distance_to(c)
					if d < closest_dist:
						closest_dist = d
				
				# Incentivizes being close to frontier boundaries.
				# Closer to 0 distance yields up to +1.0 reward.
				var proximity_shaping = 1.0 / (1.0 + closest_dist)
				#print(proximity_shaping)
				reward += proximity_shaping
		
		# 4. Completion bonus
		if coverage >= coverage_threshold:
			reward += 100.0
			print("EPISODE STEPS : ", n_steps ," , ACTIONS TAKEN : " , actions_taken)
			#print("DONE!!!!!!!!")
			done = true
			needs_reset = true
			
		previous_coverage = coverage
		
		
		#print("my step:", actions_taken,",default step :",n_steps, ", sum reward : ", reward)
		
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

	# Ignore new actions while travelling
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

	# Determine the drone's current position in grid coordinates.
	var current_grid_pos = Vector3i.ZERO
	var drone_pos = navigator.drone.global_position

	if grid_manager.has_method("world_to_grid"):
		current_grid_pos = grid_manager.world_to_grid(drone_pos)
	elif grid_manager.has_method("global_to_map"):
		current_grid_pos = grid_manager.global_to_map(drone_pos)
	elif grid_manager.has_method("local_to_map"):
		current_grid_pos = grid_manager.local_to_map(grid_manager.to_local(drone_pos))
	else:
		# Fallback approximation if no coordinate conversion helper exists
		current_grid_pos = Vector3i(
			roundi(drone_pos.x),
			roundi(drone_pos.y),
			roundi(drone_pos.z)
		)

	# Interpret action as relative displacement within a sphere
	var displacement = Vector3(act[0], act[1], act[2])
	if displacement.length() > 1.0:
		displacement = displacement.normalized()

	# Scale relative displacement by the configured radius limits
	var offset = displacement * max_action_radius
	var target_grid_pos = Vector3(current_grid_pos) + offset

	# Clamp coordinates to within grid boundaries
	var target_coord = Vector3i(
		clampi(roundi(target_grid_pos.x), 0, grid_manager.grid_size.x - 1),
		clampi(roundi(target_grid_pos.y), 0, grid_manager.grid_size.y - 1),
		clampi(roundi(target_grid_pos.z), 0, grid_manager.grid_size.z - 1)
	)
	#print("DRONE POS : " ,drone_pos)
	#print("TARGET:",target_coord)
	navigator.perform_action(target_coord)
	

# =====================================================
# RESET
# =====================================================

func reset() -> void:
	print("RESET CALLED")

	super.reset()
	needs_reset=false
	done = false
	actions_taken = 0
	previous_coverage = 0.0
	n_steps = 0
	if is_instance_valid(navigator):
		navigator.reset_rl_stats()

	if is_instance_valid(grid_manager):
		grid_manager.reset_grid()
		
	if is_instance_valid(swarm_controller):
		swarm_controller.reset_swarm_pos()

func set_done_false():
	done = false
