# ObstacleSpawner.gd
class_name ObstacleSpawner
extends Node3D

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")

@export_group("Obstacle Configuration")
@export var spawn_count: int = 8
@export var spawn_interval: float = 0.0
@export var min_obstacle_scale: float = 0.4
@export var max_obstacle_scale: float = 1.0
@export var drone_clearance: float = 2.5
@export var spawn_safe_radius: float = 3.5

@export_group("Spawn Area")
@export var spawn_box_size: Vector3 = Vector3(30, 30, 30)
@export var spawn_center: Vector3 = Vector3(15, 15, 15)

@export_group("Scenes (Assign at least one)")
@export var ground_static_scene: PackedScene
@export var ground_moving_scene: PackedScene
@export var flying_static_scene: PackedScene
@export var flying_moving_scene: PackedScene

var rng = RandomNumberGenerator.new()
var placed_obstacles: Array[Node3D] = []

func _ready() -> void:
	if grid_manager != null and "grid_size" in grid_manager:
		spawn_box_size = Vector3(grid_manager.grid_size.x, grid_manager.grid_size.y, grid_manager.grid_size.z)
		spawn_center = spawn_box_size * 0.5
	rng.randomize()
	spawn_all_obstacles()

func spawn_all_obstacles() -> void:
	# Verify at least one scene is assigned
	if not _has_any_valid_scene():
		push_error("ObstacleSpawner: No PackedScenes assigned! Please assign at least one obstacle scene in the Inspector.")
		return

	for i in range(spawn_count):
		_spawn_single_obstacle_guaranteed()
		if spawn_interval > 0:
			await get_tree().create_timer(spawn_interval).timeout

func _spawn_single_obstacle_guaranteed() -> void:
	var is_flying = rng.randf() < 0.5
	var is_moving = rng.randf() < 0.75
	
	var obstacle: Node3D = _get_scene_instance(is_flying, is_moving)
	if obstacle == null:
		return

	var category = ObstacleTag.ObstacleCategory.STATIC_STRUCTURE
	var pattern = MovingObstacle.Pattern.LINEAR_PATROL
	var move_speed: float = 1.0

	# Configure Archetypes
	if is_flying:
		if is_moving:
			if rng.randf() < 0.5:
				category = ObstacleTag.ObstacleCategory.DRONE_UAV
				pattern = MovingObstacle.Pattern.ROGUE_DRONE
				move_speed = rng.randf_range(3.0, 5.5)
			else:
				category = ObstacleTag.ObstacleCategory.BIRD_WILDLIFE
				pattern = MovingObstacle.Pattern.BIRD_SOARING
				move_speed = rng.randf_range(2.0, 4.0)
		else:
			category = ObstacleTag.ObstacleCategory.STATIC_STRUCTURE
	else:
		if is_moving:
			var roll = rng.randf()
			if roll < 0.35:
				category = ObstacleTag.ObstacleCategory.PEDESTRIAN
				pattern = MovingObstacle.Pattern.HUMAN_PEDESTRIAN
				move_speed = rng.randf_range(1.0, 1.8)
			elif roll < 0.70:
				category = ObstacleTag.ObstacleCategory.GROUND_VEHICLE
				pattern = MovingObstacle.Pattern.CAR_TRAFFIC
				move_speed = rng.randf_range(3.5, 6.0)
			else:
				category = ObstacleTag.ObstacleCategory.AGV_ROBOT
				pattern = MovingObstacle.Pattern.AGV_GRID
				move_speed = rng.randf_range(1.5, 2.5)
		else:
			category = ObstacleTag.ObstacleCategory.STATIC_STRUCTURE

	var scale_factor = rng.randf_range(min_obstacle_scale, max_obstacle_scale)
	scale_obstacle_children(obstacle, scale_factor)
	setup_obstacle_collision(obstacle)

	# Position with progressive clearance fallback
	var valid_pos = _find_valid_position_with_retry(is_flying, scale_factor)
	if valid_pos == Vector3.INF:
		obstacle.queue_free()
		push_warning("ObstacleSpawner: Map is too full to place obstacle #%d. Consider reducing clearance or obstacle sizes." % (placed_obstacles.size() + 1))
		return

	obstacle.position = valid_pos
	obstacle.add_to_group("obstacles")
	add_child(obstacle)
	placed_obstacles.append(obstacle)

	# 1. Attach SLAM Perception Tag
	var slam_tag = ObstacleTag.new()
	slam_tag.name = "ObstacleTag"
	slam_tag.category = category
	slam_tag.confidence = rng.randf_range(0.88, 0.99)
	slam_tag.bounding_radius = scale_factor * 1.5
	obstacle.add_child(slam_tag)

	# 2. Attach Movement Logic
	if is_moving:
		var mover = MovingObstacle.new()
		mover.name = "MovingObstacle"
		mover.pattern = pattern
		mover.base_speed = move_speed
		mover.bounds_min = Vector3(1.5, 0.5, 1.5)
		mover.bounds_max = spawn_box_size - Vector3(1.5, 1.5, 1.5)
		obstacle.add_child(mover)
	else:
		if grid_manager != null and grid_manager.has_method("register_obstacle"):
			grid_manager.register_obstacle(obstacle)

## Handles scene fallbacks so missing inspector slots don't break spawning
func _get_scene_instance(is_flying: bool, is_moving: bool) -> Node3D:
	var scene: PackedScene = null

	if is_flying:
		if is_moving and flying_moving_scene:
			scene = flying_moving_scene
		elif flying_static_scene:
			scene = flying_static_scene
		elif ground_moving_scene:
			scene = ground_moving_scene
		elif ground_static_scene:
			scene = ground_static_scene
	else:
		if is_moving and ground_moving_scene:
			scene = ground_moving_scene
		elif ground_static_scene:
			scene = ground_static_scene
		elif flying_moving_scene:
			scene = flying_moving_scene
		elif flying_static_scene:
			scene = flying_static_scene

	if scene != null:
		return scene.instantiate() as Node3D
	return null

func _has_any_valid_scene() -> bool:
	return ground_static_scene != null or ground_moving_scene != null or flying_static_scene != null or flying_moving_scene != null

## Tries strict clearance first, then relaxes clearance if map is dense
func _find_valid_position_with_retry(is_flying: bool, scale_factor: float) -> Vector3:
	var clearance_attempts = [drone_clearance, drone_clearance * 0.7, drone_clearance * 0.4]
	for clearance in clearance_attempts:
		var pos = _find_valid_obstacle_position(is_flying, scale_factor, clearance)
		if pos != Vector3.INF:
			return pos
	return Vector3.INF

func _find_valid_obstacle_position(is_flying: bool, scale_factor: float, clearance: float) -> Vector3:
	var max_attempts = 200
	var approx_radius = scale_factor * 1.2
	var required_dist = approx_radius + clearance

	var min_x = clearance
	var max_x = spawn_box_size.x - clearance
	var min_z = clearance
	var max_z = spawn_box_size.z - clearance
	var nfz_nodes = _get_nfz_nodes()

	for _attempt in range(max_attempts):
		var spawn_x = rng.randf_range(min_x, max_x)
		var spawn_z = rng.randf_range(min_z, max_z)
		var spawn_y: float = rng.randf_range(1.5, spawn_box_size.y - 1.5) if is_flying else 0.5

		var candidate_pos = Vector3(spawn_x, spawn_y, spawn_z)

		# Drone safe zone check (origin corner / center)
		if candidate_pos.distance_to(Vector3(1.5, 1.5, 1.5)) < (spawn_safe_radius + approx_radius):
			continue

		# Existing obstacle clearance
		var overlaps = false
		for existing in placed_obstacles:
			if is_instance_valid(existing) and candidate_pos.distance_to(existing.global_position) < required_dist:
				overlaps = true
				break
		if overlaps:
			continue

		# NFZ clearance
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

func reset_obstacles() -> void:
	for obs in placed_obstacles:
		if is_instance_valid(obs):
			obs.remove_from_group("obstacles")
			obs.queue_free()
	placed_obstacles.clear()

	var current_obstacles = get_tree().get_nodes_in_group("obstacles")
	for obs in current_obstacles:
		obs.remove_from_group("obstacles")
		obs.queue_free()

	rng.randomize()
	spawn_all_obstacles()

func setup_obstacle_collision(obstacle: Node3D) -> void:
	if obstacle is CollisionObject3D:
		obstacle.collision_layer = 0
		obstacle.collision_mask = 0
		obstacle.set_collision_layer_value(2, true) # Layer 2: Obstacles
		obstacle.set_collision_mask_value(1, true)  # Layer 1: Drones
	for child in obstacle.get_children():
		if child is CollisionObject3D:
			child.collision_layer = 0
			child.collision_mask = 0
			child.set_collision_layer_value(2, true)
			child.set_collision_mask_value(1, true)

func scale_obstacle_children(obstacle: Node3D, scale_factor: float) -> void:
	var scale_vec = Vector3.ONE * scale_factor
	for child in obstacle.get_children():
		if child is MeshInstance3D or child is CollisionShape3D:
			child.scale = scale_vec
