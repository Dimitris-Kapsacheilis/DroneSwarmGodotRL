extends Node3D
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

@export var spawn_count: int = 0
@export var spawn_interval: float = 0

# Spawn area (adjust these in the inspector)
@export var spawn_box_size: Vector3 = Vector3(50, 50, 50)  # width, height, depth
@export var spawn_center: Vector3 = Vector3.ZERO

# Obstacle scenes (assign in inspector)
@export var ground_static_scene: PackedScene
@export var ground_moving_scene: PackedScene
@export var flying_static_scene: PackedScene
@export var flying_moving_scene: PackedScene

var rng = RandomNumberGenerator.new()

func _ready():
	if grid_manager!=null:
		spawn_box_size= Vector3(grid_manager.grid_size.x,grid_manager.grid_size.y,grid_manager.grid_size.z)
		spawn_center = Vector3(spawn_box_size.x/2, spawn_box_size.y/2, spawn_box_size.z/2) 
	rng.randomize()
	spawn_all_obstacles()


func spawn_all_obstacles():
	for i in range(spawn_count):
		spawn_random_obstacle()
		await get_tree().create_timer(spawn_interval).timeout  # optional delay


func spawn_random_obstacle():
	var is_flying = rng.randf() < 0.5  # 50% chance flying
	var is_moving = rng.randf() < 0.6   # 60% chance moving
	
	var obstacle: Node3D
	
	if is_flying:
		if is_moving:
			obstacle = flying_moving_scene.instantiate()
			setup_moving_obstacle(obstacle, true)
		else:
			obstacle = flying_static_scene.instantiate()
	else:
		if is_moving:
			obstacle = ground_moving_scene.instantiate()
			setup_moving_obstacle(obstacle, false)
		else:
			obstacle = ground_static_scene.instantiate()
	
	# Random position inside box
	var pos = Vector3(
		rng.randf_range(-spawn_box_size.x/2, spawn_box_size.x/2),
		rng.randf_range(-spawn_box_size.y/2, spawn_box_size.y/2),
		rng.randf_range(-spawn_box_size.z/2, spawn_box_size.z/2)
	) + spawn_center
	
	obstacle.position = pos
	add_child(obstacle)


func setup_moving_obstacle(obstacle: Node3D, is_flying: bool):
	# Add simple movement script
	var mover = MovingObstacle.new()
	mover.is_flying = is_flying
	mover.speed = rng.randf_range(4.0, 9.0)
	mover.movement_range = rng.randf_range(8.0, 20.0)
	obstacle.add_child(mover)
