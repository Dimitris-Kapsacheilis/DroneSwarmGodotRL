extends Node3D

var zones: Array[NoFlyZone] = []

func _ready():

	var zone1 = NoFlyZone.new()
	add_child(zone1)
	zone1.setup(
		[
			Vector2(-50, -50),
			Vector2(50, -50),
			Vector2(50, 50),
			Vector2(-50, 50)
		],
		0.0,
		100.0
	)
	zones.append(zone1)

	var zone2 = NoFlyZone.new()
	add_child(zone2)
	zone2.setup(
		[
			Vector2(80, 80),
			Vector2(120, 80),
			Vector2(120, 120),
			Vector2(80, 120)
		],
		10.0,
		80.0
	)
	zones.append(zone2
	)
	var zone3 = NoFlyZone.new()
	add_child(zone3)
	zone3.setup(
		[
			Vector2(-120, 60),
			Vector2(-80, 60),
			Vector2(-80, 100),
			Vector2(-120, 100)
		],
		0.0,
		50.0
	)
	zones.append(zone3)

	#zones = [zone1, zone2, zone3]
