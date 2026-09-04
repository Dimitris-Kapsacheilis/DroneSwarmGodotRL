extends Node3D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
# 1. Strip all mesh references if running in headless training mode
	if DisplayServer.get_name() == "headless":
		_strip_meshes_for_headless(self)

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func _strip_meshes_for_headless(root_node: Node) -> void:
	var mesh_nodes = root_node.find_children("*", "MeshInstance3D", true, false)
	for mesh_node in mesh_nodes:
		mesh_node.mesh = null
