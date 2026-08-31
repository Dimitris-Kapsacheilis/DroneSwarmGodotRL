# ObstacleSpawner.gd (Attach to your obstacle spawner node)
extends Node3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

# ---------------------------------------------------------------------------
# Obstacle Configuration & Sizing
# ---------------------------------------------------------------------------
@export var spawn_count: int = 5
@export var spawn_interval: float = 0.0

## Scale range for obstacles (adjust these to change obstacle sizes)
@export var min_obstacle_scale: float = 0.1
@export var max_obstacle_scale: float = 1.0

## Minimum open corridor space between obstacles, NFZs, and map boundaries
@export var drone_clearance: float = 5.0

## Clear radius around the center/spawn point (0,0,0)
@export var spawn_safe_radius: float = 4.0

# ---------------------------------------------------------------------------
# Spawn Area
# ---------------------------------------------------------------------------
@export var spawn_box_size: Vector3 = Vector3(30, 30, 30)
@export var spawn_center: Vector3 = Vector3.ZERO

# Obstacle scenes
@export var ground_static_scene: PackedScene
@export var ground_moving_scene: PackedScene
@export var flying_static_scene: PackedScene
@export var flying_moving_scene: PackedScene

var rng = RandomNumberGenerator.new()
var placed_obstacles: Array[Node3D] = []

func _ready() -> void:
	if grid_manager != null:
		spawn_box_size = Vector3(grid_manager.grid_size.x, grid_manager.grid_size.y, grid_manager.grid_size.z)
		spawn_center = Vector3(spawn_box_size.x / 2.0, spawn_box_size.y / 2.0, spawn_box_size.z / 2.0)
	rng.randomize()
	spawn_all_obstacles()

func spawn_all_obstacles() -> void:
	for i in range(spawn_count):
		spawn_random_obstacle()
		if spawn_interval > 0:
			await get_tree().create_timer(spawn_interval).timeout

func spawn_random_obstacle() -> void:
	var is_flying = rng.randf() < 0.5
	var is_moving = rng.randf() < 0.0 # Set to > 0.0 if moving obstacles are enabled
	
	var obstacle: Node3D
	if is_flying:
		if is_moving and flying_moving_scene:
			obstacle = flying_moving_scene.instantiate()
			setup_moving_obstacle(obstacle, true)
		elif flying_static_scene:
			obstacle = flying_static_scene.instantiate()
	else:
		if is_moving and ground_moving_scene:
			obstacle = ground_moving_scene.instantiate()
			setup_moving_obstacle(obstacle, false)
		elif ground_static_scene:
			obstacle = ground_static_scene.instantiate()
			
	if obstacle == null:
		return

	# Random scale for this obstacle
	var scale_factor = rng.randf_range(min_obstacle_scale, max_obstacle_scale)
	scale_obstacle_children(obstacle, scale_factor)
	setup_obstacle_collision(obstacle)

	# Find a valid position that maintains clear flight paths
	var valid_pos = _find_valid_obstacle_position(is_flying, scale_factor)
	if valid_pos == Vector3.INF:
		obstacle.queue_free()
		return

	obstacle.position = valid_pos
	obstacle.add_to_group("obstacles")
	add_child(obstacle)
	placed_obstacles.append(obstacle)

	# Register static obstacles with the GridManager
	if not is_moving:
		if grid_manager != null and grid_manager.has_method("register_obstacle"):
			grid_manager.register_obstacle(obstacle)

# Finds a placement that does not overlap NFZs, existing obstacles, or drone spawn corridors
func _find_valid_obstacle_position(is_flying: bool, scale_factor: float) -> Vector3:
	var max_attempts = 150
	var approx_radius = scale_factor * 1.5
	var required_dist = approx_radius + drone_clearance

	var min_x = drone_clearance
	var max_x = spawn_box_size.x - drone_clearance
	var min_z = drone_clearance
	var max_z = spawn_box_size.z - drone_clearance

	var nfz_nodes = _get_nfz_nodes()

	for _attempt in range(max_attempts):
		var spawn_x = rng.randf_range(min_x, max_x)
		var spawn_z = rng.randf_range(min_z, max_z)
		var spawn_y: float = 0.0

		if is_flying:
			spawn_y = rng.randf_range(drone_clearance, spawn_box_size.y - drone_clearance)
		else:
			spawn_y = 0.5 # Ground level

		var candidate_pos = Vector3(spawn_x, spawn_y, spawn_z)

		# 1. Check clearance with Drone Spawn Safe Zone
		if candidate_pos.distance_to(Vector3(0.5, 0.5, 0.5)) < (spawn_safe_radius + required_dist):
			continue

		# 2. Check clearance with other placed obstacles
		var overlaps_obstacle = false
		for existing in placed_obstacles:
			if is_instance_valid(existing) and candidate_pos.distance_to(existing.global_position) < required_dist:
				overlaps_obstacle = true
				break
		if overlaps_obstacle:
			continue

		# 3. Check clearance with all No-Fly Zones
		var overlaps_nfz = false
		for zone in nfz_nodes:
			if _is_point_near_nfz(candidate_pos, zone, required_dist):
				overlaps_nfz = true
				break
		if overlaps_nfz:
			continue

		return candidate_pos

	return Vector3.INF

func _is_point_near_nfz(point: Vector3, zone: Node, margin: float) -> bool:
	if not ("polygon" in zone and "min_altitude" in zone and "max_altitude" in zone):
		return false
	if point.y < (zone.min_altitude - margin) or point.y > (zone.max_altitude + margin):
		return false

	var points_array = zone.polygon
	if points_array.size() == 0:
		return false

	if zone.has_method("contains_position") and zone.contains_position(point):
		return true

	var p2d = Vector2(point.x, point.z)
	var margin_sq = margin * margin
	var n = points_array.size()
	for i in range(n):
		var a = points_array[i]
		var b = points_array[(i + 1) % n]
		var ab = b - a
		var ap = p2d - a
		var l2 = ab.length_squared()
		var t = clampf(ap.dot(ab) / maxf(l2, 0.0001), 0.0, 1.0)
		var closest = a + (ab * t)
		if p2d.distance_squared_to(closest) <= margin_sq:
			return true

	return false

func _get_nfz_nodes() -> Array:
	var found: Array = []
	var root = get_tree().current_scene
	if root:
		_find_nfz_recursive(root, found)
	return found

func _find_nfz_recursive(node: Node, found: Array) -> void:
	if node is NoFlyZone:
		found.append(node)
	for child in node.get_children():
		_find_nfz_recursive(child, found)

# Call this on episode reset to remove old obstacles and spawn a fresh batch
func reset_obstacles() -> void:
	for obs in placed_obstacles:
		if is_instance_valid(obs):
			obs.remove_from_group("obstacles")
			if obs.get_parent():
				obs.get_parent().remove_child(obs)
			obs.queue_free()
	placed_obstacles.clear()

	var current_obstacles = get_tree().get_nodes_in_group("obstacles")
	for obs in current_obstacles:
		obs.remove_from_group("obstacles")
		if obs.get_parent():
			obs.get_parent().remove_child(obs)
		obs.queue_free()

	rng.randomize()
	spawn_all_obstacles()

func setup_obstacle_collision(obstacle: Node3D) -> void:
	if obstacle is CollisionObject3D:
		obstacle.collision_layer = 0
		obstacle.collision_mask = 0
		obstacle.set_collision_layer_value(2, true)
		obstacle.set_collision_mask_value(1, true)

func scale_obstacle_children(obstacle: Node3D, scale_factor: float) -> void:
	var scale_vec = Vector3(scale_factor, scale_factor, scale_factor)
	for child in obstacle.get_children():
		if child is MeshInstance3D or child is CollisionShape3D:
			child.scale = scale_vec

func setup_moving_obstacle(obstacle: Node3D, is_flying: bool) -> void:
	var mover = MovingObstacle.new()
	mover.is_flying = is_flying
	obstacle.add_child(mover)
