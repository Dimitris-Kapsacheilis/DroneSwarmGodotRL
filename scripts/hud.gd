extends Control

@onready var coveragetext = $Coverage
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	if grid_manager != null:
		coveragetext.text = "%.2f%%" % grid_manager.get_coverage_percentage()
