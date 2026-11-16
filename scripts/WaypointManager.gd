extends Node
class_name WaypointManager

enum MissionState {
	IDLE,
	WAITING_FOR_MAP,
	SENDING_WAYPOINT,
	WAITING_FOR_ARRIVAL,
	COMPLETED
}

@export var waypoint_sender: WaypointSender
@export var octomap_receiver: OctoMapReceiver

@export var waypoints: Array[Vector3] = [
	Vector3(5.0, 5.0, 0.0),
	Vector3(10.0, 5.0, 0.0),
	Vector3(10.0, 5.0, 5.0)
]

var state := MissionState.IDLE

var current_waypoint_index := 0


func start_mission() -> void:
	print("[WaypointManager] start_mission() called")
	print("[WaypointManager] State after starting: ", state)
	print("[WaypointManager] Waypoints available: ", waypoints.size())
	if waypoint_sender == null:
		push_error("WaypointSender is not assigned.")
		return

	if octomap_receiver == null:
		push_error("OctoMapReceiver is not assigned.")
		return

	if waypoints.is_empty():
		push_error("No waypoints configured.")
		return

	current_waypoint_index = 0

	state = MissionState.WAITING_FOR_MAP

	print("Mission started.")

func _process(_delta: float) -> void:

	match state:

		MissionState.WAITING_FOR_MAP:

			var bounds: AABB = octomap_receiver.get_map_bounds()

			print("[WaypointManager] Waiting for map. Bounds size: ", bounds.size)

			if bounds.size.length() > 0.001:

				print("[WaypointManager] Map ready!")
				state = MissionState.SENDING_WAYPOINT

		MissionState.SENDING_WAYPOINT:

			_send_current_waypoint()

func _send_current_waypoint() -> void:
	print("[WaypointManager] _send_current_waypoint() called")

	print("[WaypointManager] Current index: ", current_waypoint_index)
	print("[WaypointManager] Total waypoints: ", waypoints.size())

	if current_waypoint_index >= waypoints.size():
		state = MissionState.COMPLETED
		print("[WaypointManager] Mission completed.")
		return

	var target: Vector3 = waypoints[current_waypoint_index]

	print("[WaypointManager] Target: ", target)
	print("[WaypointManager] Calling waypoint sender...")

	waypoint_sender.send_waypoint(target)

	print("[WaypointManager] Waypoint sender called")

	state = MissionState.WAITING_FOR_ARRIVAL
	print("[WaypointManager] Waiting for drone to reach waypoint ", current_waypoint_index)
# Call this only after receiving actual drone-position feedback
# and verifying that the drone reached the target.
func notify_waypoint_reached() -> void:

	if state != MissionState.WAITING_FOR_ARRIVAL:
		return

	current_waypoint_index += 1

	if current_waypoint_index >= waypoints.size():

		state = MissionState.COMPLETED

		print("All waypoints reached.")

	else:

		state = MissionState.SENDING_WAYPOINT
