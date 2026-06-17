# RLBridge.gd
extends Node

@export var port: int = 11000

var server: TCPServer
var connection: StreamPeerTCP
var is_client_connected: bool = false

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
	else:
		printerr("RL Server failed to listen on port: ", port)

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
		printerr("RLBridge Error: Active drone or Navigator child node was NOT found in the scene.")
		_send_error_response("Active drone or Navigator child node was NOT found in the scene.")
		return

	match cmd.get("type"):
		"action":
			var coord = Vector3i(cmd["action"][0], cmd["action"][1], cmd["action"][2])
			print("RLBridge: New target waypoint received -> ", coord)
			nav.perform_action(coord)
			
			var start_time = Time.get_ticks_msec()
			var timeout_ms = 30000 
			
			while is_instance_valid(nav) and nav.has_target:
				await get_tree().physics_frame
				if Time.get_ticks_msec() - start_time >= timeout_ms:
					print("RLBridge: Flight timeout (30s) reached. Ending step.")
					nav.has_target = false
					break
				
			if is_instance_valid(nav):
				_send_step_data(nav)
			else:
				_send_error_response("Navigator became invalid during flight")
			
		"reset":
			nav.reset_rl_stats()
			
			# Trigger the global grid clearance
			if is_instance_valid(nav.grid_manager):
				nav.grid_manager.reset_grid()
				
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
		"done": obs["coverage"] >= 99.9,
		"error": ""
	}
	
	var json_string = JSON.stringify(response)
	var packet = json_string.to_utf8_buffer()
	connection.put_data(packet)

func _send_error_response(message: String) -> void:
	var response = {
		"position": [0.0, 0.0, 0.0],
		"velocity": [0.0, 0.0, 0.0],
		"coverage": 0.0,
		"reward": 0.0,
		"done": true,
		"error": message
	}
	var json_string = JSON.stringify(response)
	var packet = json_string.to_utf8_buffer()
	connection.put_data(packet)
