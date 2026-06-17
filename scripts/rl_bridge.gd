# RLBridge.gd
extends Node

@export var port: int = 11000

var server: TCPServer
var connection: StreamPeerTCP
var is_client_connected: bool = false

# Dynamic getter: Finds the drone via its group, then locates the child "Navigator"
var navigator: Node:
	get:
		var drones = get_tree().get_nodes_in_group("drone")
		if drones.size() > 0:
			var active_drone = drones[0]
			if is_instance_valid(active_drone):
				return active_drone.get_node_or_null("Navigator")
		return null

func _ready() -> void:
	server = TCPServer.new()
	if server.listen(port) == OK:
		print("RL Server listening on port: ", port)

func _process(_delta: float) -> void:
	if not is_client_connected:
		if server.is_connection_available():
			connection = server.take_connection()
			is_client_connected = true
			print("Python RL Client connected.")
	else:
		_process_network_messages()

func _process_network_messages() -> void:
	if connection.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		is_client_connected = false
		print("Client disconnected.")
		return

	var available_bytes = connection.get_available_bytes()
	if available_bytes > 0:
		var raw_data = connection.get_utf8_string(available_bytes)
		var json = JSON.parse_string(raw_data)
		if json:
			_handle_command(json)

func _handle_command(cmd: Dictionary) -> void:
	var nav = navigator
	if not is_instance_valid(nav):
		printerr("RLBridge Error: No active drone or Navigator child node found in the scene.")
		_send_error_response("Navigator node not found")
		return

	match cmd.get("type"):
		"action":
			var coord = Vector3i(cmd["action"][0], cmd["action"][1], cmd["action"][2])
			nav.perform_action(coord)
			
			# Wait until the drone reaches the waypoint
			while is_instance_valid(nav) and nav.has_target:
				await get_tree().physics_frame
				
			if is_instance_valid(nav):
				_send_step_data(nav)
			
		"reset":
			nav.reset_rl_stats()
			# Teleport the drone back to origin (0.5, 0.5, 0.5)
			if is_instance_valid(nav.drone):
				nav.drone.global_position = Vector3(0.5, 0.5, 0.5) 
			await get_tree().physics_frame
			_send_step_data(nav)

func _send_step_data(nav: Node) -> void:
	var obs = nav.get_observation()
	var reward = nav.get_reward()
	
	var response = {
		"position": [obs["position"].x, obs["position"].y, obs["position"].z],
		"velocity": [obs["velocity"].x, obs["velocity"].y, obs["velocity"].z],
		"coverage": obs["coverage"],
		"reward": reward,
		"done": obs["coverage"] >= 99.9
	}
	
	var json_string = JSON.stringify(response)
	connection.put_utf8_string(json_string)

func _send_error_response(message: String) -> void:
	# Sends a safe error fallback packet to Python instead of closing the connection abruptly
	var response = {
		"position": [0.0, 0.0, 0.0],
		"velocity": [0.0, 0.0, 0.0],
		"coverage": 0.0,
		"reward": 0.0,
		"done": true,
		"error": message
	}
	var json_string = JSON.stringify(response)
	connection.put_utf8_string(json_string)
