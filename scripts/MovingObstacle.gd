# MovingObstacle.gd
class_name MovingObstacle
extends Node

enum Pattern {
	CAR_TRAFFIC,       # Linear street cruise with smooth turnarounds & heading
	HUMAN_PEDESTRIAN,  # Slow wandering, waypoint pauses, natural walking wobble
	ROGUE_DRONE,       # 3D Lissajous figure-8 / spiral patrol with altitude bobbing
	BIRD_SOARING,      # Wide circular thermals with banking and altitude swoops
	AGV_GRID,          # Industrial right-angle Manhattan grid traversal
	LINEAR_PATROL      # Classic ping-pong patrol
}

@export var pattern: Pattern = Pattern.HUMAN_PEDESTRIAN
@export var base_speed: float = 2.0
@export var bounds_min: Vector3 = Vector3(1.0, 0.5, 1.0)
@export var bounds_max: Vector3 = Vector3(29.0, 20.0, 29.0)
@export var turn_speed: float = 2.0

# Runtime state
var origin: Vector3
var target_waypoint: Vector3
var wait_timer: float = 0.0
var internal_timer: float = 0.0

# Lissajous / Circular params
var orbit_radius: float = 6.0
var orbit_freq_x: float = 0.4
var orbit_freq_z: float = 0.2

# AGV grid params
var grid_step: float = 3.0
var agv_direction: Vector3 = Vector3.FORWARD

func _ready() -> void:
	var parent = get_parent() as Node3D
	if parent:
		origin = parent.global_position
		target_waypoint = origin
	internal_timer = randf() * 100.0 # Desynchronize obstacles

func _physics_process(delta: float) -> void:
	var parent = get_parent() as Node3D
	if not is_instance_valid(parent):
		return

	internal_timer += delta

	match pattern:
		Pattern.CAR_TRAFFIC:
			_process_car(parent, delta)
		Pattern.HUMAN_PEDESTRIAN:
			_process_human(parent, delta)
		Pattern.ROGUE_DRONE:
			_process_drone(parent, delta)
		Pattern.BIRD_SOARING:
			_process_bird(parent, delta)
		Pattern.AGV_GRID:
			_process_agv(parent, delta)
		Pattern.LINEAR_PATROL:
			_process_linear(parent, delta)

# ---------------------------------------------------------------------------
# 1. Car / Ground Vehicle: Moves along straight corridors and turns
# ---------------------------------------------------------------------------
func _process_car(parent: Node3D, delta: float) -> void:
	if parent.global_position.distance_to(target_waypoint) < 0.8 or target_waypoint == origin:
		var lane_axis = Vector3.FORWARD if randf() < 0.5 else Vector3.RIGHT
		if randf() < 0.5:
			lane_axis = -lane_axis
		var travel_dist = randf_range(10.0, 25.0)
		target_waypoint = _clamp_to_bounds(parent.global_position + (lane_axis * travel_dist))
		target_waypoint.y = origin.y

	var dir = (target_waypoint - parent.global_position)
	dir.y = 0.0
	if dir.length() > 0.05:
		dir = dir.normalized()
		parent.global_position += dir * base_speed * delta
		_smooth_look(parent, dir, delta)

# ---------------------------------------------------------------------------
# 2. Human / Pedestrian: Slow, pauses at intervals, wanders
# ---------------------------------------------------------------------------
func _process_human(parent: Node3D, delta: float) -> void:
	if wait_timer > 0.0:
		wait_timer -= delta
		return

	if parent.global_position.distance_to(target_waypoint) < 0.6 or target_waypoint == origin:
		if randf() < 0.35:
			wait_timer = randf_range(1.5, 3.5)
		var wander_offset = Vector3(randf_range(-6.0, 6.0), 0, randf_range(-6.0, 6.0))
		target_waypoint = _clamp_to_bounds(parent.global_position + wander_offset)
		target_waypoint.y = origin.y

	var move_dir = (target_waypoint - parent.global_position)
	move_dir.y = 0.0
	if move_dir.length() > 0.05:
		move_dir = move_dir.normalized()
		var wobble = sin(internal_timer * 4.0) * 0.06
		var forward = move_dir + Vector3(wobble, 0, -wobble)
		parent.global_position += forward.normalized() * base_speed * delta
		_smooth_look(parent, move_dir, delta)

# ---------------------------------------------------------------------------
# 3. Rogue Drone: 3D Lissajous curve with continuous altitude variation
# ---------------------------------------------------------------------------
func _process_drone(parent: Node3D, delta: float) -> void:
	var next_x = origin.x + sin(internal_timer * orbit_freq_x) * orbit_radius
	var next_z = origin.z + sin(internal_timer * orbit_freq_z * 2.0) * (orbit_radius * 0.7)
	var next_y = origin.y + sin(internal_timer * 1.8) * 1.5

	var target = _clamp_to_bounds(Vector3(next_x, next_y, next_z))
	var dir = (target - parent.global_position)
	parent.global_position = target
	
	if dir.length() > 0.01:
		_smooth_look(parent, dir.normalized(), delta)

# ---------------------------------------------------------------------------
# 4. Soaring Bird: Circular circling pattern with swoops
# ---------------------------------------------------------------------------
func _process_bird(parent: Node3D, delta: float) -> void:
	var angle = internal_timer * (base_speed / orbit_radius)
	var target_x = origin.x + cos(angle) * orbit_radius
	var target_z = origin.z + sin(angle) * orbit_radius
	var target_y = origin.y + sin(angle * 2.0) * 1.8

	var next_pos = _clamp_to_bounds(Vector3(target_x, target_y, target_z))
	var fly_dir = (next_pos - parent.global_position)
	parent.global_position = next_pos
	
	if fly_dir.length() > 0.01:
		_smooth_look(parent, fly_dir.normalized(), delta)

# ---------------------------------------------------------------------------
# 5. AGV / Warehouse Robot: Strict 90-degree Manhattan grid traversal
# ---------------------------------------------------------------------------
func _process_agv(parent: Node3D, delta: float) -> void:
	if parent.global_position.distance_to(target_waypoint) < 0.15 or target_waypoint == origin:
		parent.global_position = target_waypoint
		var possible_dirs = [Vector3.FORWARD, Vector3.BACK, Vector3.LEFT, Vector3.RIGHT]
		possible_dirs.erase(-agv_direction)
		agv_direction = possible_dirs[randi() % possible_dirs.size()]
		
		var candidate = parent.global_position + (agv_direction * grid_step)
		if _is_inside_bounds(candidate):
			target_waypoint = candidate
		else:
			agv_direction = -agv_direction
			target_waypoint = parent.global_position + (agv_direction * grid_step)
		target_waypoint.y = origin.y

	parent.global_position = parent.global_position.move_toward(target_waypoint, base_speed * delta)
	_smooth_look(parent, agv_direction, delta)

# ---------------------------------------------------------------------------
# 6. Linear Patrol
# ---------------------------------------------------------------------------
func _process_linear(parent: Node3D, delta: float) -> void:
	if parent.global_position.distance_to(target_waypoint) < 0.5 or target_waypoint == origin:
		target_waypoint = origin + Vector3(randf_range(-12.0, 12.0), 0, randf_range(-12.0, 12.0))
		target_waypoint = _clamp_to_bounds(target_waypoint)
	
	var dir = (target_waypoint - parent.global_position).normalized()
	parent.global_position += dir * base_speed * delta
	_smooth_look(parent, dir, delta)

# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------
func _smooth_look(parent: Node3D, direction: Vector3, delta: float) -> void:
	if direction.length_squared() < 0.0001:
		return
	var target_yaw = atan2(-direction.x, -direction.z)
	parent.rotation.y = lerp_angle(parent.rotation.y, target_yaw, turn_speed * delta)

func _clamp_to_bounds(pos: Vector3) -> Vector3:
	return Vector3(
		clampf(pos.x, bounds_min.x, bounds_max.x),
		clampf(pos.y, bounds_min.y, bounds_max.y),
		clampf(pos.z, bounds_min.z, bounds_max.z)
	)

func _is_inside_bounds(pos: Vector3) -> bool:
	return pos.x >= bounds_min.x and pos.x <= bounds_max.x and \
		   pos.y >= bounds_min.y and pos.y <= bounds_max.y and \
		   pos.z >= bounds_min.z and pos.z <= bounds_max.z
