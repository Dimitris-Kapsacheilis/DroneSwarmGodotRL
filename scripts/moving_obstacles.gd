# MovingObstacle.gd
extends Node
class_name MovingObstacle

@export var speed: float = 6.0
@export var movement_range: float = 15.0
@export var is_flying: bool = false

var start_pos: Vector3
var time_offset: float

func _ready():
	start_pos = get_parent().position
	time_offset = randf() * 10.0  # random phase so they don't move in sync


func _physics_process(delta):
	var parent = get_parent() as Node3D
	if not parent:
		return
	
	var offset = sin(Time.get_ticks_msec() * 0.001 + time_offset) * movement_range * 0.5
	
	if is_flying:
		# Flying: side to side + slight up/down
		parent.position.x = start_pos.x + offset
		parent.position.y = start_pos.y + sin(Time.get_ticks_msec() * 0.002 + time_offset) * 3.0
	else:
		# Ground: left-right or forward movement
		parent.position.x = start_pos.x + offset
		# Optional: add forward movement
		# parent.position.z += speed * delta * 0.5
