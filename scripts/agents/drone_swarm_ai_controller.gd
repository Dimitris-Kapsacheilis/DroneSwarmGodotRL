#class_name DroneSwarmAIController
extends Node3D
#
#enum ControlModes {
	#INHERIT_FROM_SYNC,
	#HUMAN,
	#TRAINING,
	#ONNX_INFERENCE,
	#RECORD_EXPERT_DEMOS
#}
#
#@export var control_mode: ControlModes = ControlModes.TRAINING
#@export var onnx_model_path := ""
#@export var reset_after := 180000
#@export var action_repeat: int = 1
#@export var policy_name: String = "shared_policy"
#@export var rl_environment_path: NodePath = NodePath("../RL Environment")
#
#var heuristic := "human"
#var done := false
#var reward := 0.0
#var n_steps := 0
#var needs_reset := false
#var onnx_model = null
#var _environment: DroneSwarmRLEnvironment = null
#var _last_action: Array[float] = []
#
#func _ready() -> void:
	#add_to_group("AGENT")
	#_environment = get_node_or_null(rl_environment_path) as DroneSwarmRLEnvironment
	#if _environment == null:
		#push_error("DroneSwarmAIController cannot find RL Environment at %s" % rl_environment_path)
#
#func _physics_process(_delta: float) -> void:
	#n_steps += 1
	#if n_steps > reset_after:
		#needs_reset = true
#
	#if needs_reset:
		#reset()
#
	#if _environment:
		#done = _environment.done
#
#func init(_player: Node3D) -> void:
	#pass
#
#func get_obs() -> Dictionary:
	#if _environment == null:
		#return {"obs": []}
#
	#if needs_reset:
		#reset()
#
	#return {"obs": _environment.get_flat_observation()}
#
#func get_reward() -> float:
	#if _environment == null:
		#return 0.0
#
	#reward = _environment.get_reward()
	#return reward
#
#func get_action_space() -> Dictionary:
	#return {
		#"task_assignments": {
			#"size": _action_size(),
			#"action_type": "continuous"
		#}
	#}
#
#func set_action(action = null) -> void:
	#if _environment == null:
		#return
#
	#var values := _extract_action_values(action)
	#_last_action = values
	#_environment.apply_flat_action(values)
#
#func get_action() -> Array[float]:
	#return _last_action.duplicate()
#
#func get_info() -> Dictionary:
	#if _environment == null:
		#return {}
#
	#var drones := _environment._get_drones()
	#return {
		#"is_success": _environment.mission_complete,
		#"episode": _environment.episode_index,
		#"elapsed_seconds": _environment.episode_time,
		#"drone_count": drones.size(),
		#"task_count": _environment.get_task_count(),
		#"coverage_fraction": _environment.get_coverage_fraction(),
		#"target_known_fraction": _environment.get_target_known_fraction(),
		#"target_tracked_fraction": _environment.get_target_tracked_fraction()
	#}
#
#func get_obs_space() -> Dictionary:
	#return {
		#"obs": {
			#"size": [get_obs()["obs"].size()],
			#"space": "box"
		#}
	#}
#
#func reset() -> void:
	#n_steps = 0
	#needs_reset = false
	#done = false
	#reward = 0.0
	#if _environment:
		#_environment.reset()
#
#func reset_if_done() -> void:
	#if done:
		#reset()
#
#func set_heuristic(next_heuristic: String) -> void:
	#heuristic = next_heuristic
#
#func get_done() -> bool:
	#return done
#
#func set_done_false() -> void:
	#done = false
	#if _environment:
		#_environment.done = false
#
#func zero_reward() -> void:
	#reward = 0.0
#
#func _action_size() -> int:
	#if _environment == null:
		#return 0
	#return _environment.get_assignment_action_size()
#
#func _extract_action_values(action: Variant) -> Array[float]:
	#var values: Array[float] = []
#
	#if action is Dictionary and action.has("task_assignments"):
		#for value in action["task_assignments"]:
			#values.append(float(value))
	#elif action is Dictionary and action.has("drone_actions"):
		#for value in action["drone_actions"]:
			#values.append(float(value))
	#elif action is Array:
		#for value in action:
			#values.append(float(value))
#
	#while values.size() < _action_size():
		#values.append(0.0)
#
	#while values.size() > _action_size():
		#values.pop_back()
#
	#for i in range(values.size()):
		#values[i] = clamp(values[i], -1.0, 1.0)
#
	#return values
