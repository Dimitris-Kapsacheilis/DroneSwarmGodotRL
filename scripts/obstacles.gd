# ObstacleSpawner.gd (Attach to your obstacle spawner node)
extends Node3D
@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

@export var spawn_count: int = 0
@export var spawn_interval: float = 0
@export var obstacle_scale: float = 1.0

# Spawn area
@export var spawn_box_size: Vector3 = Vector3(50, 50, 50)
@export var spawn_center: Vector3 = Vector3.ZERO

# Obstacle scenes
@export var ground_static_scene: PackedScene
@export var ground_moving_scene: PackedScene
@export var flying_static_scene: PackedScene
@export var flying_moving_scene: PackedScene

var rng = RandomNumberGenerator.new()

func _ready():
	if grid_manager != null:
		spawn_box_size = Vector3(grid_manager.grid_size.x, grid_manager.grid_size.y, grid_manager.grid_size.z)
		spawn_center = Vector3(spawn_box_size.x/2, spawn_box_size.y/2, spawn_box_size.z/2) 
	rng.randomize()
	spawn_all_obstacles()


func spawn_all_obstacles():
	for i in range(spawn_count):
		spawn_random_obstacle()
		if spawn_interval > 0:
			await get_tree().create_timer(spawn_interval).timeout


func spawn_random_obstacle():
	#Chances of obstacle of different type
	var is_flying = rng.randf() < 0.5
	var is_moving = rng.randf() < 0.99
	
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
	
	# Determine Y position
	var spawn_y: float = 0.0
	if is_flying:
		spawn_y = rng.randf_range(-spawn_box_size.y/2, spawn_box_size.y/2) + spawn_center.y
	else:
		spawn_y = spawn_center.y - (spawn_box_size.y / 2)
	
	# Random X and Z positions
	var pos = Vector3(
		rng.randf_range(-spawn_box_size.x/2, spawn_box_size.x/2) + spawn_center.x,
		spawn_y,
		rng.randf_range(-spawn_box_size.z/2, spawn_box_size.z/2) + spawn_center.z
	)
	
	obstacle.position = pos
	
	# Set up collision masking and scaling
	setup_obstacle_collision(obstacle)
	scale_obstacle_children(obstacle, obstacle_scale)
	
	# --- IMPORTANT: Add the obstacle to the group so the AI finds it ---
	obstacle.add_to_group("obstacles")
	
	add_child(obstacle)
# --- ONLY REGISTER STATIC OBSTACLES TO THE GRID MANAGER ---
	if not is_moving:
		if grid_manager != null and grid_manager.has_method("register_obstacle"):
			grid_manager.register_obstacle(obstacle)


# Call this to safely delete old obstacles and spawn new ones
func reset_obstacles():
	# 1. Find and clear all existing obstacles safely
	var current_obstacles = get_tree().get_nodes_in_group("obstacles")
	for obs in current_obstacles:
		obs.remove_from_group("obstacles") # Remove from tracking group immediately
		obs.queue_free()                  # Queue deletion at end of frame
	
	# 2. Re-randomize seed so the new layout is different
	rng.randomize()
	
	# 3. Spawn a fresh batch of obstacles
	spawn_all_obstacles()


func setup_obstacle_collision(obstacle: Node3D):
	if obstacle is CollisionObject3D:
		obstacle.collision_layer = 0
		obstacle.collision_mask = 0
		obstacle.set_collision_layer_value(2, true)
		obstacle.set_collision_mask_value(1, true)


func scale_obstacle_children(obstacle: Node3D, scale_factor: float):
	var scale_vec = Vector3(scale_factor, scale_factor, scale_factor)
	for child in obstacle.get_children():
		if child is MeshInstance3D or child is CollisionShape3D:
			child.scale = scale_vec


func setup_moving_obstacle(obstacle: Node3D, is_flying: bool):
	var mover = MovingObstacle.new()
	mover.is_flying = is_flying
	#mover.speed = rng.randf_range(4.0, 9.0)
	#mover.movement_range = rng.randf_range(8.0, 20.0)
	obstacle.add_child(mover)
