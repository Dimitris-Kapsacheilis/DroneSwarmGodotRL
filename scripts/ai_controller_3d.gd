extends AIController3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var navigator = get_parent().get_node_or_null("Navigator")
@onready var swarm_controller = get_parent().get_node_or_null("/root/Swarm Test/Swarm Controller")

var episode_steps := 0
var previous_coverage := 0.0


func _ready() -> void:
	super._ready()


# =====================================================
# OBSERVATIONS
# =====================================================

func get_obs() -> Dictionary:
	if not is_instance_valid(navigator) or not is_instance_valid(navigator.drone):
		return {
			"obs": [
				0.0, 0.0, 0.0,
				0.0, 0.0, 0.0,
				0.0
			]
		}

	var pos = navigator.drone.global_position
	var vel = navigator.velocity

	var coverage := 0.0
	if is_instance_valid(grid_manager):
		coverage = grid_manager.get_coverage_percentage() / 100.0

	return {
		"obs": [
			pos.x,
			pos.y,
			pos.z,
			vel.x,
			vel.y,
			vel.z,
			coverage
		]
	}


# =====================================================
# REWARD
# =====================================================

func get_reward() -> float:
	if not is_instance_valid(grid_manager):
		return 0.0

	var coverage = grid_manager.get_coverage_percentage() / 100.0

	# Reward only newly discovered coverage
	var reward = (coverage - previous_coverage) * 10.0

	# Small living penalty
	reward -= 0.001

	# Large completion bonus
	if coverage >= 0.70:
		reward += 50.0

	previous_coverage = coverage

	return reward


# =====================================================
# TERMINATION
# =====================================================

func get_done() -> bool:
	if not is_instance_valid(grid_manager):
		return false

	#var coverage = grid_manager.get_coverage_percentage()
#
	#if coverage >= 50.0:
		#print("DONE")
		#done = true
		#return true
#
	#if needs_reset:
#
		#print("TIMEOUT")
		#done = true
		#return true



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

	if not is_instance_valid(navigator):
		return

	if not is_instance_valid(grid_manager):
		return

	# Ignore new actions while travelling
	if navigator.has_target:
		#print("HAS TARGET")
		return

	var act = []

	if action is Dictionary:
		if not action.has("flight_waypoint"):
			return

		act = action["flight_waypoint"]
	else:
		act = action

	if act.size() < 3:
		return

	episode_steps += 1

	var target_x = lerp(
		0.0,
		float(grid_manager.grid_size.x - 1),
		(act[0] + 1.0) / 2.0
	)

	var target_y = lerp(
		0.0,
		float(grid_manager.grid_size.y - 1),
		(act[1] + 1.0) / 2.0
	)

	var target_z = lerp(
		0.0,
		float(grid_manager.grid_size.z - 1),
		(act[2] + 1.0) / 2.0
	)

	var target_coord = Vector3i(
		roundi(target_x),
		roundi(target_y),
		roundi(target_z)
	)

	navigator.perform_action(target_coord)


# =====================================================
# RESET
# =====================================================

func reset() -> void:
	print("RESET CALLED")

	super.reset()

	done = false

	episode_steps = 0
	previous_coverage = 0.0

	if is_instance_valid(navigator):
		navigator.reset_rl_stats()

		#if is_instance_valid(navigator.drone):
			#navigator.drone.global_position = Vector3(0.5,0.5,0.5)

	if is_instance_valid(grid_manager):
		grid_manager.reset_grid()
		
	if is_instance_valid(swarm_controller):
		swarm_controller.reset_swarm_pos()



func set_done_false():
	done = false
# =====================================================
# DEBUG
# =====================================================

#func _process(_delta: float) -> void:
	#if episode_steps % 100 == 0 and episode_steps > 0:
		#if is_instance_valid(grid_manager):
			#print(
				#"Steps: ",
				#episode_steps,
				#" Coverage: ",
				#grid_manager.get_coverage_percentage(),
				#"%"
			#)
