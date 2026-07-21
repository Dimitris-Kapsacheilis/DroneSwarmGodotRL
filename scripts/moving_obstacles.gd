# MovingObstacle.gd
extends Node
class_name MovingObstacle

@export var speed: float = 0.5           # units/sec, constant travel speed
@export var movement_range: float = 35.0  # total horizontal distance covered (peak-to-peak)
@export var is_flying: bool = false
@export var vertical_speed: float = 0.5   # units/sec for vertical bob (flying only)
@export var vertical_range: float = 25.0

@export var forward_speed: float = 0.0    # units/sec, constant forward creep (0 = disabled)

var start_pos: Vector3
var t: float        # 0..1 ping-pong progress, horizontal
var t_dir: float = 1.0
var vt: float        # 0..1 ping-pong progress, vertical
var vt_dir: float = 1.0

func _ready():
	start_pos = get_parent().position
	# randomize starting point so obstacles don't sync up
	t = randf()
	vt = randf()

func _physics_process(delta):
	var parent = get_parent() as Node3D
	if not parent:
		return

	# advance horizontal ping-pong progress at constant speed
	if movement_range > 0.0:
		var t_speed = speed / movement_range   # convert units/sec -> progress/sec
		t += t_speed * t_dir * delta
		if t >= 1.0:
			t = 1.0
			t_dir = -1.0
		elif t <= 0.0:
			t = 0.0
			t_dir = 1.0

	var offset = lerp(-movement_range * 0.5, movement_range * 0.5, t)
	parent.position.x = start_pos.x + offset

	if is_flying:
		if vertical_range > 0.0:
			var vt_speed = vertical_speed / vertical_range
			vt += vt_speed * vt_dir * delta
			if vt >= 1.0:
				vt = 1.0
				vt_dir = -1.0
			elif vt <= 0.0:
				vt = 0.0
				vt_dir = 1.0
		var v_offset = lerp(-vertical_range * 0.5, vertical_range * 0.5, vt)
		parent.position.y = start_pos.y + v_offset

	if forward_speed != 0.0:
		parent.position.z -= forward_speed * delta
