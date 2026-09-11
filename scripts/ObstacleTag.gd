# ObstacleTag.gd
class_name ObstacleTag
extends Node

enum ObstacleCategory {
	UNKNOWN,
	STATIC_STRUCTURE,     # Towers, pillars, buildings, trees
	PEDESTRIAN,           # Walking humans (1.0 - 1.8 m/s)
	GROUND_VEHICLE,       # Cars, trucks (4.0 - 10.0 m/s)
	AGV_ROBOT,            # Warehouse AGVs, forklifts (1.5 - 2.5 m/s)
	DRONE_UAV,            # Rogue non-cooperative drones (3.0 - 7.0 m/s)
	BIRD_WILDLIFE         # Birds, flocks (2.0 - 5.0 m/s)
}

@export var category: ObstacleCategory = ObstacleCategory.UNKNOWN
@export var confidence: float = 0.95
@export var bounding_radius: float = 1.0

var track_id: int = 0
var last_position: Vector3 = Vector3.ZERO
var estimated_velocity: Vector3 = Vector3.ZERO

static var _next_id: int = 1000

func _ready() -> void:
	track_id = _next_id
	_next_id += 1
	var parent = get_parent() as Node3D
	if parent:
		last_position = parent.global_position

func _physics_process(delta: float) -> void:
	if delta <= 0.0:
		return
	var parent = get_parent() as Node3D
	if not is_instance_valid(parent):
		return
		
	var current_pos = parent.global_position
	# Compute instantaneous linear velocity for SLAM/EKF tracking
	estimated_velocity = (current_pos - last_position) / delta
	last_position = current_pos

## Returns the SLAM telemetry payload dictionary for drone sensor fusion
func get_slam_payload() -> Dictionary:
	var parent = get_parent() as Node3D
	var pos = parent.global_position if parent else Vector3.ZERO
	return {
		"id": track_id,
		"category": ObstacleCategory.keys()[category],
		"category_id": int(category),
		"position": pos,
		"velocity": estimated_velocity,
		"speed": estimated_velocity.length(),
		"bounding_radius": bounding_radius,
		"confidence": confidence,
		"timestamp": Time.get_ticks_msec() / 1000.0
	}
