class_name DroneSwarmRLEnvironment
extends Node

signal step_collected(result: Dictionary)
const Drone = preload("res://scripts/drone.gd")
const SwarmController = preload("res://scripts/swarm_controller.gd")

@export var swarm_controller_path: NodePath = NodePath("../Swarm Controller")
@export var target_waypoint_index: int = 0
@export var episode_seconds: float = 30.0
@export var success_radius: float = 5.0
@export var max_distance_from_origin: float = 250.0
@export var telemetry_enabled: bool = true
@export var max_telemetry_steps: int = 2000
@export var position_scale: float = 100.0
@export var velocity_scale: float = 25.0

var swarm_controller: SwarmController = null
var episode_time: float = 0.0
var episode_index: int = 0
var done: bool = false
var telemetry: Array[Dictionary] = []

func _ready() -> void:
	swarm_controller = get_node_or_null(swarm_controller_path) as SwarmController
	if swarm_controller == null:
		push_error("RL Environment cannot find swarm controller at %s" % swarm_controller_path)

func _physics_process(delta: float) -> void:
	if done or swarm_controller == null:
		return

	episode_time += delta
	done = _compute_done()

func reset(random_seed: int = 0) -> Dictionary:
	if swarm_controller == null:
		return {}

	if random_seed != 0:
		seed(random_seed)

	episode_index += 1
	episode_time = 0.0
	done = false
	telemetry.clear()

	var drones: Array[Drone] = _get_drones()
	var count: int = max(drones.size(), 1)
	for i in range(drones.size()):
		var x: float = (i - float(count - 1) / 2.0) * swarm_controller.follow_spread * 2.0
		var pos: Vector3 = Vector3(x, swarm_controller.spawn_height, 0.0)
		drones[i].reset_flight_state(pos)

	swarm_controller.individual_mode = false
	swarm_controller.waypoint_mode = false
	swarm_controller.selected_drone_index = 0
	swarm_controller.set_leader(0)
	swarm_controller.set_formation(swarm_controller.current_formation)

	return collect_step_result()

func step(actions: Array) -> Dictionary:
	apply_actions(actions)
	return collect_step_result()

func apply_actions(actions: Array) -> void:
	var drones := _get_drones()
	for i in range(min(drones.size(), actions.size())):
		drones[i].set_rl_action(_action_to_vector4(actions[i]))

func apply_flat_action(action_values: Array) -> void:
	var actions: Array[Vector4] = []
	var drones := _get_drones()
	for i in range(drones.size()):
		var base := i * 4
		if base + 3 >= action_values.size():
			actions.append(Vector4.ZERO)
			continue

		actions.append(Vector4(
			float(action_values[base]),
			float(action_values[base + 1]),
			float(action_values[base + 2]),
			float(action_values[base + 3])
		))

	apply_actions(actions)

func collect_step_result() -> Dictionary:
	var result: Dictionary = {
		"observation": get_observation(),
		"reward": get_reward(),
		"done": done,
		"episode": episode_index,
		"elapsed_seconds": episode_time
	}
	_record_telemetry(result)
	step_collected.emit(result)
	return result

func get_telemetry() -> Array[Dictionary]:
	return telemetry.duplicate(true)

func clear_telemetry() -> void:
	telemetry.clear()

func get_observation() -> Dictionary:
	var drone_observations: Array[Dictionary] = []
	for drone in _get_drones():
		drone_observations.append(drone.get_rl_observation())

	return {
		"target": _vector3_to_array(_target_position()),
		"formation": swarm_controller.current_formation if swarm_controller else "",
		"drones": drone_observations
	}

func get_flat_observation() -> Array[float]:
	var obs: Array[float] = []
	var drones: Array[Drone] = _get_drones()
	var target: Vector3 = _target_position()
	var leader: Drone = swarm_controller.current_leader if swarm_controller and swarm_controller.current_leader else null
	var leader_position: Vector3 = leader.global_position if leader else Vector3.ZERO
	var remaining_time: float = 1.0 - clamp(episode_time / max(episode_seconds, 0.001), 0.0, 1.0)
	var target_index_norm: float = 0.0
	if swarm_controller:
		target_index_norm = float(target_waypoint_index) / max(float(max(swarm_controller.waypoints.size() - 1, 1)), 1.0)

	obs.append(remaining_time)
	obs.append(target_index_norm)

	for drone in drones:
		var relative_target: Vector3 = (target - drone.global_position) / position_scale
		var relative_leader: Vector3 = (leader_position - drone.global_position) / position_scale
		var linear: Vector3 = drone.linear_velocity / velocity_scale
		var angular: Vector3 = drone.angular_velocity / velocity_scale

		_append_vector3(obs, relative_target)
		_append_vector3(obs, relative_leader)
		_append_vector3(obs, linear)
		_append_vector3(obs, angular)
		obs.append(1.0 if drone.is_leader else 0.0)

	return obs

func get_reward() -> float:
	var drones: Array[Drone] = _get_drones()
	if drones.is_empty():
		return -1.0

	var target: Vector3 = _target_position()
	var leader: Drone = swarm_controller.current_leader if swarm_controller.current_leader else drones[0]
	var leader_distance: float = leader.global_position.distance_to(target)
	var reward: float = -leader_distance * 0.01

	var average_spacing_error: float = 0.0
	if drones.size() > 1:
		for drone in drones:
			if drone == leader:
				continue
			average_spacing_error += abs(drone.global_position.distance_to(leader.global_position) - swarm_controller.follow_distance)
		average_spacing_error /= float(drones.size() - 1)
		reward -= average_spacing_error * 0.02

	if leader_distance <= success_radius:
		reward += 10.0

	if _is_out_of_bounds():
		reward -= 10.0

	for drone in drones:
		reward -= drone.linear_velocity.length() * 0.001

	return reward

func _compute_done() -> bool:
	var drones := _get_drones()
	if drones.is_empty():
		return true

	var leader: Drone = swarm_controller.current_leader if swarm_controller.current_leader else drones[0]
	if leader.global_position.distance_to(_target_position()) <= success_radius:
		return true

	return episode_time >= episode_seconds or _is_out_of_bounds()

func _is_out_of_bounds() -> bool:
	for drone in _get_drones():
		if drone.global_position.length() > max_distance_from_origin:
			return true
	return false

func _target_position() -> Vector3:
	if swarm_controller == null or swarm_controller.waypoints.is_empty():
		return Vector3.ZERO

	var idx: int = clamp(target_waypoint_index, 0, swarm_controller.waypoints.size() - 1)
	return swarm_controller.waypoints[idx]

func _get_drones() -> Array[Drone]:
	if swarm_controller == null:
		return []
	return swarm_controller.drones

func _action_to_vector4(action: Variant) -> Vector4:
	if action is Vector4:
		return action

	if action is Array and action.size() >= 4:
		return Vector4(float(action[0]), float(action[1]), float(action[2]), float(action[3]))

	if action is Dictionary:
		return Vector4(
			float(action.get("strafe", 0.0)),
			float(action.get("lift", 0.0)),
			float(action.get("forward", 0.0)),
			float(action.get("yaw", 0.0))
		)

	return Vector4.ZERO

func _vector3_to_array(value: Vector3) -> Array[float]:
	return [value.x, value.y, value.z]

func _record_telemetry(result: Dictionary) -> void:
	if not telemetry_enabled:
		return

	telemetry.append({
		"episode": result["episode"],
		"elapsed_seconds": result["elapsed_seconds"],
		"reward": result["reward"],
		"done": result["done"],
		"target": result["observation"]["target"],
		"drone_count": result["observation"]["drones"].size()
	})

	while telemetry.size() > max_telemetry_steps:
		telemetry.pop_front()

func _append_vector3(output: Array[float], value: Vector3) -> void:
	output.append(value.x)
	output.append(value.y)
	output.append(value.z)
