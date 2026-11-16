extends Node3D

@onready var waypoint_manager: WaypointManager = $WaypointManager

func _input(event: InputEvent) -> void:
	if event is InputEventKey:
		if event.pressed and not event.echo and event.keycode == KEY_M:
			print("[Main] Starting mission")
			waypoint_manager.start_mission()
