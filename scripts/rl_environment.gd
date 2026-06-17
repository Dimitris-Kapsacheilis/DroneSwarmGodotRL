class_name DroneSwarmRLEnvironment
extends Node

signal step_collected(result: Dictionary)

const Drone = preload("res://scripts/drone.gd")
const SwarmController = preload("res://scripts/swarm_controller.gd")

enum MissionTaskType {
	COVERAGE,
	TRACK_TARGET,
	RALLY
}

@export var swarm_controller_path: NodePath = NodePath("../Swarm Controller")
@export var no_fly_zone_manager_path: NodePath = NodePath("../NoFlyZoneManager")
@export var episode_seconds: float = 120.0
@export var max_distance_from_origin: float = 260.0

@export_group("Mission Area")
@export var coverage_columns: int = 4
@export var coverage_rows: int = 4
@export var coverage_area_min: Vector2 = Vector2(-180.0, -160.0)
@export var coverage_area_max: Vector2 = Vector2(180.0, 160.0)
@export var coverage_altitude: float = 105.0
@export var coverage_visit_radius: float = 30.0
@export var drone_sensor_radius: float = 45.0
@export var minimum_safe_altitude: float = 15.0
@export var maximum_safe_altitude: float = 135.0

@export_group("Targets")
@export var target_count: int = 2
@export var target_altitude: float = 65.0
@export var target_sensor_radius: float = 55.0
@export var target_follow_radius: float = 35.0
@export var max_target_memory_seconds: float = 18.0

@export_group("Observation Scaling")
@export var position_scale: float = 200.0
@export var velocity_scale: float = 25.0

@export_group("Reward")
@export var coverage_reward: float = 3.0
@export var coverage_progress_reward: float = 0.15
@export var full_coverage_bonus: float = 25.0
@export var target_detection_reward: float = 8.0
@export var target_tracking_reward: float = 0.6
@export var target_stale_penalty: float = 0.4
@export var mission_complete_bonus: float = 40.0
@export var duplicate_assignment_penalty: float = 0.25
@export var assignment_change_penalty: float = 0.08
@export var no_fly_zone_penalty: float = 20.0
@export var out_of_bounds_penalty: float = 25.0
@export var altitude_penalty_scale: float = 0.08

@export_group("Telemetry")
@export var telemetry_enabled: bool = true
@export var max_telemetry_steps: int = 2000

var swarm_controller: SwarmController = null
var no_fly_zone_manager: Node = null
var episode_time: float = 0.0
var episode_index: int = 0
var done: bool = false
var mission_complete: bool = false
var telemetry: Array[Dictionary] = []

var coverage_cells: Array[Dictionary] = []
var targets: Array[Dictionary] = []
var mission_tasks: Array[Dictionary] = []
var last_assignments: Array[int] = []
var assigned_task_counts: Array[int] = []
var rally_position: Vector3 = Vector3.ZERO

var last_reward: float = 0.0
var new_coverage_count: int = 0
var new_target_detection_count: int = 0
var tracked_target_count: int = 0
var no_fly_zone_violations: int = 0
var assignment_change_count: int = 0
var duplicate_assignment_count: int = 0
var full_coverage_awarded: bool = false

func _ready() -> void:
	swarm_controller = get_node_or_null(swarm_controller_path) as SwarmController
	if not is_instance_valid(swarm_controller):
		push_error("RL Environment cannot find swarm controller at %s" % swarm_controller_path)

	no_fly_zone_manager = get_node_or_null(no_fly_zone_manager_path)
	_rebuild_mission_state()


func _physics_process(delta: float) -> void:
	if done or not is_instance_valid(swarm_controller):
		return

	episode_time += delta
	_update_targets(delta)
	_update_mission_sensing()
	_update_mission_complete()
	var out_of_bounds: bool = _is_out_of_bounds()
	last_reward += _compute_reward(out_of_bounds)
	done = _compute_done(out_of_bounds)
	assignment_change_count = 0


func reset(random_seed: int = 0) -> Dictionary:
	if not is_instance_valid(swarm_controller):
		return {}

	if random_seed != 0:
		seed(random_seed)

	episode_index += 1
	episode_time = 0.0
	done = false
	mission_complete = false
	last_reward = 0.0
	assignment_change_count = 0
	duplicate_assignment_count = 0
	full_coverage_awarded = false
	telemetry.clear()
	_rebuild_mission_state()

	var drones: Array[Drone] = _get_drones()
	var count: int = max(drones.size(), 1)
	for i in range(drones.size()):
		if not is_instance_valid(drones[i]):
			continue
		var x: float = (i - float(count - 1) / 2.0) * swarm_controller.follow_spread * 2.0
		var pos: Vector3 = Vector3(x + coverage_area_min.x, coverage_altitude, coverage_area_min.y - 35.0)
		drones[i].reset_flight_state(pos)

	swarm_controller.individual_mode = true
	swarm_controller.waypoint_mode = false
	swarm_controller.selected_drone_index = 0
	swarm_controller.leader_index = 0
	swarm_controller.current_leader = drones[0] if not drones.is_empty() else null
	for drone in drones:
		if is_instance_valid(drone):
			drone.is_leader = (drone == swarm_controller.current_leader)

	_reset_assignments_to_rally()
	_update_mission_sensing()
	return collect_step_result()


func step(actions: Array) -> Dictionary:
	apply_actions(actions)
	return collect_step_result()


func apply_actions(actions: Array) -> void:
	var values: Array[float] = []
	var task_count: int = max(mission_tasks.size(), 1)
	for action in actions:
		var task_index: int = 0
		if action is Dictionary:
			task_index = int(action.get("task", 0))
		else:
			task_index = int(action)

		task_index = clamp(task_index, 0, task_count - 1)
		for i in range(task_count):
			values.append(1.0 if i == task_index else -1.0)

	apply_flat_action(values)


func apply_flat_action(action_values: Array) -> void:
	var drones: Array[Drone] = _get_drones()
	if drones.is_empty() or mission_tasks.is_empty():
		return

	_ensure_assignment_arrays()
	assignment_change_count = 0

	var task_count: int = max(mission_tasks.size(), 1)
	for i in range(drones.size()):
		var task_index: int = _action_scores_to_task_index(action_values, i, task_count)
		if last_assignments[i] != task_index:
			assignment_change_count += 1
		last_assignments[i] = task_index
		_command_drone_to_task(drones[i], task_index)

	_update_assignment_counts()


func collect_step_result() -> Dictionary:
	var result: Dictionary = {
		"observation": get_observation(),
		"reward": last_reward,
		"done": done,
		"episode": episode_index,
		"elapsed_seconds": episode_time
	}
	last_reward = 0.0
	_record_telemetry(result)
	step_collected.emit(result)
	return result


func get_telemetry() -> Array[Dictionary]:
	return telemetry.duplicate(true)


func clear_telemetry() -> void:
	telemetry.clear()


func get_observation() -> Dictionary:
	return {
		"mission": {
			"coverage_fraction": get_coverage_fraction(),
			"target_known_fraction": get_target_known_fraction(),
			"target_tracked_fraction": get_target_tracked_fraction(),
			"task_count": mission_tasks.size()
		},
		"coverage_cells": coverage_cells.duplicate(true),
		"targets": _get_target_observations(),
		"assignments": last_assignments.duplicate(),
		"drones": _get_drone_observations()
	}


func get_flat_observation() -> Array[float]:
	var obs: Array[float] = []
	var drones: Array[Drone] = _get_drones()
	var swarm_center: Vector3 = _swarm_center()
	var remaining_time: float = 1.0 - clamp(episode_time / max(episode_seconds, 0.001), 0.0, 1.0)

	obs.append(remaining_time)
	obs.append(get_coverage_fraction())
	obs.append(get_target_known_fraction())
	obs.append(get_target_tracked_fraction())
	obs.append(float(no_fly_zone_violations) / float(max(drones.size(), 1)))

	for drone_index in range(drones.size()):
		var drone: Drone = drones[drone_index]
		if not is_instance_valid(drone):
			continue
		var assignment: int = _assignment_for_drone(drone_index)
		var task_position: Vector3 = _task_position(assignment)
		_append_vector3(obs, (drone.global_position - swarm_center) / position_scale)
		_append_vector3(obs, drone.linear_velocity / velocity_scale)
		_append_vector3(obs, (task_position - drone.global_position) / position_scale)
		_append_vector3(obs, _nearest_no_fly_zone_vector(drone.global_position) / position_scale)
		obs.append(_normalized_task_index(assignment))
		obs.append(_normalized_altitude(drone.global_position.y))
		obs.append(1.0 if _is_in_no_fly_zone(drone.global_position) else 0.0)

	for i in range(coverage_cells.size()):
		var cell: Dictionary = coverage_cells[i]
		var center: Vector3 = cell["center"]
		_append_vector3(obs, (center - swarm_center) / position_scale)
		obs.append(1.0 if cell["covered"] else 0.0)
		obs.append(1.0 if _cell_recently_visible(cell) else 0.0)
		obs.append(1.0 if cell["blocked"] else 0.0)
		obs.append(float(_assigned_count_for_coverage_cell(i)) / float(max(drones.size(), 1)))

	for target_index in range(targets.size()):
		var target: Dictionary = targets[target_index]
		var memory_position: Vector3 = _target_memory_position(target_index)
		var target_age: float = _target_memory_age(target_index)
		_append_vector3(obs, (memory_position - swarm_center) / position_scale)
		obs.append(1.0 if _target_is_known(target_index) else 0.0)
		obs.append(clamp(target_age / max(max_target_memory_seconds, 0.001), 0.0, 1.0))
		obs.append(1.0 if target["tracked"] else 0.0)
		obs.append(float(_assigned_count_for_target(target_index)) / float(max(drones.size(), 1)))

	return obs


func get_reward() -> float:
	return last_reward


func get_assignment_action_size() -> int:
	return _get_drones().size() * max(mission_tasks.size(), 1)


func get_task_count() -> int:
	return mission_tasks.size()


func get_coverage_fraction() -> float:
	var coverable_count: int = 0
	var covered_count: int = 0
	for cell in coverage_cells:
		if cell["blocked"]:
			continue
		coverable_count += 1
		if cell["covered"]:
			covered_count += 1

	if coverable_count == 0:
		return 1.0
	return float(covered_count) / float(coverable_count)


func get_target_known_fraction() -> float:
	if targets.is_empty():
		return 1.0

	var known_count: int = 0
	for i in range(targets.size()):
		if _target_is_known(i):
			known_count += 1
	return float(known_count) / float(targets.size())


func get_target_tracked_fraction() -> float:
	if targets.is_empty():
		return 1.0
	return float(tracked_target_count) / float(targets.size())


func _rebuild_mission_state() -> void:
	coverage_cells.clear()
	targets.clear()
	mission_tasks.clear()
	rally_position = Vector3(coverage_area_min.x, coverage_altitude, coverage_area_min.y - 20.0)
	_build_coverage_grid()
	_initialize_targets()
	_build_mission_tasks()
	_ensure_assignment_arrays()
	_update_assignment_counts()


func _build_coverage_grid() -> void:
	var columns: int = max(coverage_columns, 1)
	var rows: int = max(coverage_rows, 1)

	for row in range(rows):
		for col in range(columns):
			var x_ratio: float = (float(col) + 0.5) / float(columns)
			var z_ratio: float = (float(row) + 0.5) / float(rows)
			var center_x: float = lerp(coverage_area_min.x, coverage_area_max.x, x_ratio)
			var center_z: float = lerp(coverage_area_min.y, coverage_area_max.y, z_ratio)
			var center: Vector3 = Vector3(center_x, coverage_altitude, center_z)
			coverage_cells.append({
				"center": center,
				"covered": false,
				"last_seen_time": -INF,
				"blocked": _is_in_no_fly_zone(center),
				"assigned_count": 0
			})


func _initialize_targets() -> void:
	var count: int = max(target_count, 0)
	for i in range(count):
		var search_position: Vector3 = _safe_target_search_position(i)
		targets.append({
			"position": search_position,
			"center": search_position,
			"radius": 18.0 + float(i) * 4.0,
			"phase": randf_range(0.0, TAU),
			"speed": 0.45 + float(i) * 0.12,
			"search_position": search_position,
			"known": false,
			"last_seen_position": search_position,
			"last_seen_time": -INF,
			"currently_visible": false,
			"tracked": false,
			"assigned_count": 0
		})


func _build_mission_tasks() -> void:
	for i in range(coverage_cells.size()):
		if coverage_cells[i]["blocked"]:
			continue
		mission_tasks.append({
			"type": MissionTaskType.COVERAGE,
			"index": i
		})

	for i in range(targets.size()):
		mission_tasks.append({
			"type": MissionTaskType.TRACK_TARGET,
			"index": i
		})

	mission_tasks.append({
		"type": MissionTaskType.RALLY,
		"index": 0
	})


func _reset_assignments_to_rally() -> void:
	_ensure_assignment_arrays()
	var rally_task: int = max(mission_tasks.size() - 1, 0)
	var drones: Array[Drone] = _get_drones()
	for i in range(last_assignments.size()):
		last_assignments[i] = rally_task
		if i < drones.size() and is_instance_valid(drones[i]):
			_command_drone_to_task(drones[i], rally_task)
	_update_assignment_counts()


func _ensure_assignment_arrays() -> void:
	var drone_count: int = _get_drones().size()
	while last_assignments.size() < drone_count:
		last_assignments.append(max(mission_tasks.size() - 1, 0))
	while last_assignments.size() > drone_count:
		last_assignments.pop_back()

	while assigned_task_counts.size() < mission_tasks.size():
		assigned_task_counts.append(0)
	while assigned_task_counts.size() > mission_tasks.size():
		assigned_task_counts.pop_back()


func _update_targets(delta: float) -> void:
	for i in range(targets.size()):
		var target: Dictionary = targets[i]
		var phase: float = float(target["phase"]) + float(target["speed"]) * delta
		var center: Vector3 = target["center"]
		var radius: float = target["radius"]
		target["phase"] = phase
		target["position"] = center + Vector3(cos(phase) * radius, 0.0, sin(phase) * radius)
		targets[i] = target


func _update_mission_sensing() -> void:
	new_coverage_count = 0
	new_target_detection_count = 0
	tracked_target_count = 0
	no_fly_zone_violations = 0

	for i in range(coverage_cells.size()):
		coverage_cells[i]["assigned_count"] = _assigned_count_for_coverage_cell(i)

	for i in range(targets.size()):
		targets[i]["currently_visible"] = false
		targets[i]["tracked"] = false
		targets[i]["assigned_count"] = _assigned_count_for_target(i)

	for drone in _get_drones():
		if not is_instance_valid(drone):
			continue
		if _is_in_no_fly_zone(drone.global_position):
			no_fly_zone_violations += 1

		for cell_index in range(coverage_cells.size()):
			var cell: Dictionary = coverage_cells[cell_index]
			if cell["blocked"]:
				continue

			var distance: float = drone.global_position.distance_to(cell["center"])
			if distance <= drone_sensor_radius:
				cell["last_seen_time"] = episode_time
			if distance <= coverage_visit_radius and not cell["covered"]:
				cell["covered"] = true
				cell["last_seen_time"] = episode_time
				new_coverage_count += 1
			coverage_cells[cell_index] = cell

		for target_index in range(targets.size()):
			var target: Dictionary = targets[target_index]
			var distance_to_target: float = drone.global_position.distance_to(target["position"])
			if distance_to_target <= target_sensor_radius:
				if not _target_is_known(target_index):
					new_target_detection_count += 1
				target["known"] = true
				target["currently_visible"] = true
				target["last_seen_position"] = target["position"]
				target["last_seen_time"] = episode_time
			if distance_to_target <= target_follow_radius:
				target["tracked"] = true
			targets[target_index] = target

	for i in range(targets.size()):
		if targets[i]["tracked"]:
			tracked_target_count += 1

	_update_assignment_counts()


func _update_assignment_counts() -> void:
	_ensure_assignment_arrays()
	for i in range(assigned_task_counts.size()):
		assigned_task_counts[i] = 0

	for task_index in last_assignments:
		if task_index >= 0 and task_index < assigned_task_counts.size():
			assigned_task_counts[task_index] += 1

	duplicate_assignment_count = 0
	for count in assigned_task_counts:
		if count > 1:
			duplicate_assignment_count += count - 1


func _update_mission_complete() -> void:
	mission_complete = get_coverage_fraction() >= 0.999 and get_target_tracked_fraction() >= 0.999


func _compute_reward(out_of_bounds: bool = false) -> float:
	var reward: float = 0.0
	reward += float(new_coverage_count) * coverage_reward
	reward += get_coverage_fraction() * coverage_progress_reward
	reward += float(new_target_detection_count) * target_detection_reward
	reward += float(tracked_target_count) * target_tracking_reward

	for i in range(targets.size()):
		if _target_is_known(i) and not targets[i]["tracked"]:
			reward -= _target_memory_age(i) / max(max_target_memory_seconds, 0.001) * target_stale_penalty

	if get_coverage_fraction() >= 0.999 and not full_coverage_awarded:
		reward += full_coverage_bonus
		full_coverage_awarded = true
	if mission_complete:
		reward += mission_complete_bonus

	reward -= float(duplicate_assignment_count) * duplicate_assignment_penalty
	reward -= float(assignment_change_count) * assignment_change_penalty
	reward -= float(no_fly_zone_violations) * no_fly_zone_penalty

	if out_of_bounds:
		reward -= out_of_bounds_penalty

	for drone in _get_drones():
		if is_instance_valid(drone):
			reward -= _altitude_penalty(drone.global_position.y)

	return reward


func _compute_done(out_of_bounds: bool = false) -> bool:
	return mission_complete or episode_time >= episode_seconds or out_of_bounds


func _command_drone_to_task(drone: Drone, task_index: int) -> void:
	if not is_instance_valid(drone):
		return
	if mission_tasks.is_empty():
		drone.clear_targets()
		return

	var safe_index: int = clamp(task_index, 0, mission_tasks.size() - 1)
	drone.go_to_waypoint(_task_position(safe_index), -1)


func _task_position(task_index: int) -> Vector3:
	if mission_tasks.is_empty():
		return rally_position

	var task: Dictionary = mission_tasks[clamp(task_index, 0, mission_tasks.size() - 1)]
	match int(task["type"]):
		MissionTaskType.COVERAGE:
			return coverage_cells[int(task["index"])]["center"]
		MissionTaskType.TRACK_TARGET:
			return _target_command_position(int(task["index"]))
		MissionTaskType.RALLY:
			return rally_position
		_:
			return rally_position


func _target_command_position(target_index: int) -> Vector3:
	if target_index < 0 or target_index >= targets.size():
		return rally_position

	if _target_is_known(target_index):
		return targets[target_index]["last_seen_position"]
	return targets[target_index]["search_position"]


func _target_memory_position(target_index: int) -> Vector3:
	if target_index < 0 or target_index >= targets.size():
		return rally_position
	if _target_is_known(target_index):
		return targets[target_index]["last_seen_position"]
	return targets[target_index]["search_position"]


func _target_memory_age(target_index: int) -> float:
	if target_index < 0 or target_index >= targets.size():
		return max_target_memory_seconds

	var last_seen_time: float = targets[target_index]["last_seen_time"]
	if is_inf(last_seen_time):
		return max_target_memory_seconds
	return max(episode_time - last_seen_time, 0.0)


func _target_is_known(target_index: int) -> bool:
	if target_index < 0 or target_index >= targets.size():
		return false
	if not targets[target_index]["known"]:
		return false
	return _target_memory_age(target_index) <= max_target_memory_seconds


func _action_scores_to_task_index(action_values: Array, drone_index: int, task_count: int) -> int:
	var base: int = drone_index * task_count
	if base >= action_values.size():
		return max(mission_tasks.size() - 1, 0)

	var best_index: int = max(mission_tasks.size() - 1, 0)
	var best_score: float = -INF

	for task_index in range(task_count):
		var flat_index: int = base + task_index
		var score: float = -1.0
		if flat_index < action_values.size():
			score = clamp(float(action_values[flat_index]), -1.0, 1.0)
		if score > best_score:
			best_score = score
			best_index = task_index

	return clamp(best_index, 0, task_count - 1)


func _assignment_for_drone(drone_index: int) -> int:
	if drone_index < 0 or drone_index >= last_assignments.size():
		return max(mission_tasks.size() - 1, 0)
	return last_assignments[drone_index]


func _normalized_task_index(task_index: int) -> float:
	if mission_tasks.size() <= 1:
		return 0.0
	return float(clamp(task_index, 0, mission_tasks.size() - 1)) / float(mission_tasks.size() - 1)


func _normalized_altitude(altitude: float) -> float:
	return clamp(inverse_lerp(minimum_safe_altitude, maximum_safe_altitude, altitude), -1.0, 1.0)


func _assigned_count_for_coverage_cell(cell_index: int) -> int:
	var count: int = 0
	for assignment in last_assignments:
		if assignment < 0 or assignment >= mission_tasks.size():
			continue
		var task: Dictionary = mission_tasks[assignment]
		if int(task["type"]) == MissionTaskType.COVERAGE and int(task["index"]) == cell_index:
			count += 1
	return count


func _assigned_count_for_target(target_index: int) -> int:
	var count: int = 0
	for assignment in last_assignments:
		if assignment < 0 or assignment >= mission_tasks.size():
			continue
		var task: Dictionary = mission_tasks[assignment]
		if int(task["type"]) == MissionTaskType.TRACK_TARGET and int(task["index"]) == target_index:
			count += 1
	return count


func _cell_recently_visible(cell: Dictionary) -> bool:
	var last_seen_time: float = cell["last_seen_time"]
	if is_inf(last_seen_time):
		return false
	return episode_time - last_seen_time <= 3.0


func _safe_target_search_position(index: int) -> Vector3:
	if coverage_cells.is_empty():
		return Vector3(rally_position.x, target_altitude, rally_position.z)

	var safe_index: int = index % coverage_cells.size()
	for i in range(coverage_cells.size()):
		var cell: Dictionary = coverage_cells[(safe_index + i) % coverage_cells.size()]
		var candidate: Vector3 = cell["center"]
		candidate.y = target_altitude
		if not _is_in_no_fly_zone(candidate):
			return candidate

	var fallback: Vector3 = coverage_cells[safe_index]["center"]
	fallback.y = target_altitude
	return fallback


func _swarm_center() -> Vector3:
	var drones: Array[Drone] = _get_drones()
	if drones.is_empty():
		return rally_position

	var center: Vector3 = Vector3.ZERO
	var valid_count: int = 0
	for drone in drones:
		if is_instance_valid(drone):
			center += drone.global_position
			valid_count += 1
	return center / float(max(valid_count, 1))


func _get_drone_observations() -> Array[Dictionary]:
	var observations: Array[Dictionary] = []
	var drones: Array[Drone] = _get_drones()
	for drone_index in range(drones.size()):
		var drone: Drone = drones[drone_index]
		if not is_instance_valid(drone):
			continue
		observations.append({
			"id": drone.drone_id,
			"position": _vector3_to_array(drone.global_position),
			"linear_velocity": _vector3_to_array(drone.linear_velocity),
			"assignment": _assignment_for_drone(drone_index),
			"task_position": _vector3_to_array(_task_position(_assignment_for_drone(drone_index))),
			"in_no_fly_zone": _is_in_no_fly_zone(drone.global_position)
		})
	return observations


func _get_target_observations() -> Array[Dictionary]:
	var observations: Array[Dictionary] = []
	for target_index in range(targets.size()):
		var target: Dictionary = targets[target_index]
		var target_obs: Dictionary = {
			"id": target_index,
			"known": _target_is_known(target_index),
			"memory_position": _vector3_to_array(_target_memory_position(target_index)),
			"memory_age": _target_memory_age(target_index),
			"currently_visible": target["currently_visible"],
			"tracked": target["tracked"],
			"assigned_count": target["assigned_count"]
		}
		if target["currently_visible"]:
			target_obs["visible_position"] = _vector3_to_array(target["position"])
		observations.append(target_obs)
	return observations


func _is_out_of_bounds() -> bool:
	for drone in _get_drones():
		if is_instance_valid(drone) and drone.global_position.length() > max_distance_from_origin:
			return true
	return false


func _is_in_no_fly_zone(pos: Vector3) -> bool:
	for zone in _get_no_fly_zones():
		if is_instance_valid(zone) and zone.has_method("contains_position"):
			if zone.contains_position(pos):
				return true
	return false


func _nearest_no_fly_zone_vector(pos: Vector3) -> Vector3:
	var zones: Array = _get_no_fly_zones()
	if zones.is_empty():
		return Vector3.ZERO

	var best: Vector3 = Vector3.ZERO
	var best_distance: float = INF
	for zone in zones:
		if is_instance_valid(zone):
			var center: Vector3 = _zone_center(zone)
			var to_zone: Vector3 = center - pos
			var distance: float = to_zone.length()
			if distance < best_distance:
				best_distance = distance
				best = to_zone
	return best


func _zone_center(zone: Object) -> Vector3:
	if not is_instance_valid(zone):
		return Vector3.ZERO
		
	var polygon = zone.get("polygon")
	if polygon == null or polygon.is_empty():
		return Vector3.ZERO

	var center_2d: Vector2 = Vector2.ZERO
	for point in polygon:
		center_2d += point
	center_2d /= float(polygon.size())
	
	var min_altitude: float = float(zone.get("min_altitude")) if zone.get("min_altitude") != null else 0.0
	var max_altitude: float = float(zone.get("max_altitude")) if zone.get("max_altitude") != null else 0.0
	
	return Vector3(center_2d.x, (min_altitude + max_altitude) * 0.5, center_2d.y)


func _altitude_penalty(altitude: float) -> float:
	if altitude < minimum_safe_altitude:
		return (minimum_safe_altitude - altitude) * altitude_penalty_scale
	if altitude > maximum_safe_altitude:
		return (altitude - maximum_safe_altitude) * altitude_penalty_scale
	return 0.0


func _get_no_fly_zones() -> Array:
	if not is_instance_valid(no_fly_zone_manager):
		return []

	var zones = no_fly_zone_manager.get("zones")
	return zones if zones is Array else []


func _get_drones() -> Array[Drone]:
	var active_drones: Array[Drone] = []
	if is_instance_valid(swarm_controller) and swarm_controller.drones is Array:
		for item in swarm_controller.drones:
			if is_instance_valid(item) and item is Drone:
				active_drones.append(item)
	return active_drones


func _vector3_to_array(value: Vector3) -> Array[float]:
	return [value.x, value.y, value.z]


func _append_vector3(output: Array[float], value: Vector3) -> void:
	output.append(value.x)
	output.append(value.y)
	output.append(value.z)


func _record_telemetry(result: Dictionary) -> void:
	if not telemetry_enabled:
		return

	telemetry.append({
		"episode": result["episode"],
		"elapsed_seconds": result["elapsed_seconds"],
		"reward": result["reward"],
		"done": result["done"],
		"coverage_fraction": get_coverage_fraction(),
		"target_known_fraction": get_target_known_fraction(),
		"target_tracked_fraction": get_target_tracked_fraction(),
		"task_count": mission_tasks.size(),
		"drone_count": _get_drones().size()
	})

	while telemetry.size() > max_telemetry_steps:
		telemetry.pop_front()
