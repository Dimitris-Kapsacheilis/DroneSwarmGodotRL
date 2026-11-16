extends Node3D
class_name OctoMapReceiver

@export var listen_port: int = 5005

@export var overview_camera: Camera3D

@export var voxel_color: Color = Color(0.25, 0.75, 1.0)

@export_range(1, 1000000)
var max_voxels_to_render: int = 200000

@export var camera_distance_multiplier: float = 1.8
@export var camera_height_ratio: float = 0.8

var udp := PacketPeerUDP.new()

var active_map_id: int = -1
var expected_chunks: int = 0

var received_chunks: Dictionary = {}

var multimesh_instance: MultiMeshInstance3D
var multimesh: MultiMesh

var camera_initialized := false
var current_map_bounds := AABB()
var current_voxel_resolution: float = 1.0

func _ready() -> void:

	var error := udp.bind(listen_port, "0.0.0.0")

	if error != OK:
		push_error("Could not bind OctoMap UDP port: %s" % error)
		return

	print("OctoMap receiver listening on port ", listen_port)

	_create_voxel_renderer()


func _create_voxel_renderer() -> void:

	multimesh = MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true

	var box := BoxMesh.new()
	box.size = Vector3.ONE

	multimesh.mesh = box

	multimesh_instance = MultiMeshInstance3D.new()
	multimesh_instance.multimesh = multimesh

	var material := StandardMaterial3D.new()
	material.albedo_color = voxel_color
	material.shading_mode = BaseMaterial3D.SHADING_MODE_PER_PIXEL

	multimesh_instance.material_override = material

	add_child(multimesh_instance)


var debug_packets := 0

func _process(_delta: float) -> void:

	while udp.get_available_packet_count() > 0:

		var packet := udp.get_packet()
		var json_text := packet.get_string_from_utf8()

		debug_packets += 1

		if debug_packets <= 5:
			print("[OctoMapReceiver] Raw packet: ", json_text)

		var data = JSON.parse_string(json_text)

		if data == null:
			print("[OctoMapReceiver] JSON parsing failed")
			continue

		if not data is Dictionary:
			print("[OctoMapReceiver] JSON is not a Dictionary")
			continue

		if debug_packets <= 5:
			print("[OctoMapReceiver] Packet type: ", data.get("type", "MISSING"))
			print("[OctoMapReceiver] Keys: ", data.keys())

		if data.get("type", "") != "octomap_chunk":
			print("[OctoMapReceiver] Unexpected packet type")
			continue

		_consume_chunk(data)

# ROS ENU: X East, Y North, Z Up.
# Godot: X right, Y up, Z forward.
func ros_to_godot(p: Vector3) -> Vector3:

	return Vector3(
		p.x,
		p.z,
		-p.y
	)


func _consume_chunk(data: Dictionary) -> void:

	var map_id := int(data.get("map_id", -1))
	var chunk := int(data.get("chunk", -1))
	var chunks := int(data.get("chunks", 0))
	if chunk == 0:
		current_voxel_resolution = float(data.get("resolution", 1.0))

	if map_id < 0 or chunk < 0 or chunks <= 0:
		return

	if map_id < active_map_id:
		return

	if map_id > active_map_id:

		active_map_id = map_id
		expected_chunks = chunks
		received_chunks.clear()

	if chunks != expected_chunks:
		return

	if received_chunks.has(chunk):
		return

	var rows = data.get("voxels", [])

	if not rows is Array:
		return

	received_chunks[chunk] = rows

	if received_chunks.size() == expected_chunks:
		_render_complete_map()

func _render_complete_map() -> void:

	var voxels: Array[Dictionary] = []

	# The C++ bridge sends resolution separately.
	var voxel_size: float = current_voxel_resolution

	# Get resolution from the received map data.
	# All chunks belonging to this map use the same resolution.
	# We store it when receiving the first chunk.
	for chunk_index in range(expected_chunks):

		if not received_chunks.has(chunk_index):
			return

		var rows: Array = received_chunks[chunk_index]

		for row in rows:

			if not row is Array or row.size() < 3:
				continue

			if voxels.size() >= max_voxels_to_render:
				break

			var ros_position := Vector3(
				float(row[0]),
				float(row[1]),
				float(row[2])
			)

			var godot_position := ros_to_godot(ros_position)

			voxels.append({
				"position": godot_position,
				"size": voxel_size
			})

	if voxels.is_empty():
		print("[OctoMapReceiver] No valid voxels found.")
		return

	_render_voxels(voxels)

	if not camera_initialized:
		_frame_camera()

	print(
		"Rendered map ",
		active_map_id,
		" with ",
		voxels.size(),
		" occupied voxels"
	)

func _render_voxels(voxels: Array[Dictionary]) -> void:

	multimesh.instance_count = voxels.size()

	var first := true
	var bounds := AABB()

	for i in range(voxels.size()):

		var voxel: Dictionary = voxels[i]

		var position: Vector3 = voxel["position"]
		var size: float = voxel["size"]

		var transform := Transform3D(
			Basis.IDENTITY.scaled(Vector3.ONE * size),
			position
		)

		multimesh.set_instance_transform(i, transform)
		multimesh.set_instance_color(i, voxel_color)

		var voxel_bounds := AABB(
			position - Vector3.ONE * size * 0.5,
			Vector3.ONE * size
		)

		if first:
			bounds = voxel_bounds
			first = false
		else:
			bounds = bounds.merge(voxel_bounds)

	current_map_bounds = bounds


func _frame_camera() -> void:

	if overview_camera == null:
		push_warning("Assign an OverviewCamera to OctoMapReceiver.")
		return

	if current_map_bounds.size.length() <= 0.001:
		return

	var center := current_map_bounds.get_center()

	var extent := current_map_bounds.size

	var max_extent := maxf(
		extent.x,
		maxf(extent.y, extent.z)
	)

	var distance := max_extent * camera_distance_multiplier + 5.0

	var direction := Vector3(
		1.0,
		camera_height_ratio,
		1.0
	).normalized()

	overview_camera.global_position = center + direction * distance

	overview_camera.look_at(center, Vector3.UP)

	overview_camera.make_current()

	camera_initialized = true


func get_map_bounds() -> AABB:
	return current_map_bounds
