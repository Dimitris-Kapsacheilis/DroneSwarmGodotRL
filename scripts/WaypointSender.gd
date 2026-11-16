extends Node
class_name WaypointSender

@export var receiver_ip: String = "127.0.0.1"
@export var receiver_port: int = 5006

var udp: PacketPeerUDP


func _ready() -> void:
	udp = PacketPeerUDP.new()

	var error = udp.set_dest_address(receiver_ip, receiver_port)

	if error != OK:
		push_error("[WaypointSender] Failed to configure UDP destination: " + str(error))
		return

	print("[WaypointSender] Ready. Sending to ", receiver_ip, ":", receiver_port)


# Convert Godot coordinates to ROS coordinates.
# Assumes ROS: X East, Y North, Z Up.
func godot_to_ros(position: Vector3) -> Vector3:
	return Vector3(position.x, -position.z, position.y)


func send_waypoint(position: Vector3) -> void:
	var ros_position = godot_to_ros(position)

	# IMPORTANT: The receiver requires type = "waypoint".
	var data = {
		"type": "waypoint",
		"x": ros_position.x,
		"y": ros_position.y,
		"z": ros_position.z,
		"frame_id": "map"
	}

	var packet = JSON.stringify(data).to_utf8_buffer()

	var error = udp.put_packet(packet)

	if error == OK:
		print("[WaypointSender] Sent waypoint: ", data)
	else:
		push_error("[WaypointSender] UDP send failed: " + str(error))
