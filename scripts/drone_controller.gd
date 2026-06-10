extends AIController3D

# Stores the action sampled for the agent's policy, running in python
var move_action : float = 0.0

func get_obs() -> Dictionary:
	# get the balls position and velocity in the paddle's frame of reference
	var drone_pos = to_local(global_position)
	var obs = [drone_pos]

	return {"obs":obs}

func get_reward() -> float:	
	return reward
	
func get_action_space() -> Dictionary:
	return {
		"move_action" : {
			"size": 1,
			"action_type": "continuous"
		},
		}
	
func set_action(action) -> void:	
	move_action = clamp(action["move_action"][0], -1.0, 1.0)
