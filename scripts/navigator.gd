extends Node3D

@export var flight_speed: float = 100.0
@export var arrival_threshold: float = 0.5
@export var rl_mode: bool = true 

@onready var grid_manager = get_node_or_null("/root/Swarm Test/GridManager")
@onready var drone: Node3D = get_parent()

# Path tracking
var current_path: Array[Vector3] = []
var current_target_pos: Vector3 = Vector3.ZERO
var has_target: bool = false

# RL State Tracking
var actions_taken: int = 0
var last_position: Vector3 = Vector3.ZERO
var velocity: Vector3 = Vector3.ZERO

# Visualizer variables
var sphere_inst: MeshInstance3D
var line_inst: MeshInstance3D
var line_mesh: ImmediateMesh

func _ready() -> void:
	randomize()
	if is_instance_valid(drone):
		last_position = drone.global_position
		
		# Automatically disable gravity and physics forces if the drone is a RigidBody3D
		if drone is RigidBody3D:
			drone.freeze = true
			print("Navigator: Automatically froze RigidBody3D drone to prevent gravity fighting.")
			
	await get_tree().process_frame
	setup_visualizers()

func _physics_process(delta: float) -> void:
	if not is_instance_valid(grid_manager) or not is_instance_valid(drone):
		return

	# Calculate velocity
	velocity = (drone.global_position - last_position) / delta
	last_position = drone.global_position

	# Auto-run if not in RL mode
	if not rl_mode and not has_target:
		choose_new_random_target()

	if has_target:
		fly_along_path(delta)
		draw_trajectory_line()

func setup_visualizers() -> void:
	if not is_instance_valid(drone):
		return
		
	var world_root = drone.get_parent()
	if not world_root:
		return

	var sphere_mesh = SphereMesh.new()
	sphere_mesh.radius = 0.35
	sphere_mesh.height = 0.7
	
	var sphere_material = StandardMaterial3D.new()
	sphere_material.albedo_color = Color.GREEN
	sphere_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	sphere_inst = MeshInstance3D.new()
	sphere_inst.mesh = sphere_mesh
	sphere_inst.material_override = sphere_material
	sphere_inst.visible = false
	world_root.add_child(sphere_inst)

	var line_material = StandardMaterial3D.new()
	line_material.albedo_color = Color.RED
	line_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	line_mesh = ImmediateMesh.new()
	line_inst = MeshInstance3D.new()
	line_inst.mesh = line_mesh
	line_inst.material_override = line_material
	world_root.add_child(line_inst)

# ==========================================
# REINFORCEMENT LEARNING API
# ==========================================

func perform_action(target_coord: Vector3i) -> void:
	if not is_instance_valid(grid_manager) or not is_instance_valid(drone):
		return
		
	var clamped_coord = Vector3i(
		clamped(target_coord.x, 0, grid_manager.grid_size.x - 1),
		clamped(target_coord.y, 0, grid_manager.grid_size.y - 1),
		clamped(target_coord.z, 0, grid_manager.grid_size.z - 1)
	)

	# Get start position in grid coordinates
	var start_grid_pos = _get_current_grid_pos()
	
	# Calculate path using A*
	var grid_path = grid_manager.find_path(start_grid_pos, clamped_coord)
	
	current_path.clear()
	
	if grid_path.size() > 1:
		# Convert grid path indices back to global coordinates
		for cell in grid_path:
			var local_target = Vector3(cell.x + 0.5, cell.y + 0.5, cell.z + 0.5)
			current_path.append(grid_manager.to_global(local_target))
		
		# Set initial checkpoint
		current_target_pos = current_path[0]
		has_target = true
	else:
		# Fallback if A* found no path (or start is already end)
		var local_target = Vector3(clamped_coord.x + 0.5, clamped_coord.y + 0.5, clamped_coord.z + 0.5)
		current_target_pos = grid_manager.to_global(local_target)
		current_path = [current_target_pos]
		has_target = true

	actions_taken += 1

	if is_instance_valid(sphere_inst) and current_path.size() > 0:
		# Place visual indicator at ultimate destination
		sphere_inst.global_position = current_path[-1]
		sphere_inst.visible = true

func get_observation() -> Dictionary:
	var coverage = 0.0
	if is_instance_valid(grid_manager):
		coverage = grid_manager.get_coverage_percentage()

	var drone_pos = Vector3.ZERO
	if is_instance_valid(drone):
		drone_pos = drone.global_position

	return {
		"position": drone_pos,
		"velocity": velocity,
		"coverage": coverage
	}

func reset_rl_stats() -> void:
	actions_taken = 0
	has_target = false
	current_path.clear()
	if is_instance_valid(sphere_inst):
		sphere_inst.visible = false
	if is_instance_valid(line_mesh):
		line_mesh.clear_surfaces()

# ==========================================
# NAVIGATION & MOVEMENT
# ==========================================

func _get_current_grid_pos() -> Vector3i:
	if not is_instance_valid(drone) or not is_instance_valid(grid_manager):
		return Vector3i.ZERO
	var drone_pos = drone.global_position
	if grid_manager.has_method("world_to_grid"):
		return grid_manager.world_to_grid(drone_pos)
	elif grid_manager.has_method("global_to_map"):
		return grid_manager.global_to_map(drone_pos)
	else:
		return Vector3i(
			floor(drone_pos.x),
			floor(drone_pos.y),
			floor(drone_pos.z)
		)

func choose_new_random_target() -> void:
	var bounds = grid_manager.grid_size
	var random_coord = Vector3i(
		randi_range(0, bounds.x - 1),
		randi_range(0, bounds.y - 1),
		randi_range(0, bounds.z - 1)
	)
	perform_action(random_coord)

func fly_along_path(delta: float) -> void:
	if not is_instance_valid(drone) or current_path.is_empty():
		has_target = false
		return
		
	# Target the first node in our path queue
	current_target_pos = current_path[0]
	
	var direction = (current_target_pos - drone.global_position)
	var distance = direction.length()

	# Check if we reached the current checkpoint
	if distance <= arrival_threshold:
		current_path.remove_at(0) # Pop current waypoint
		if current_path.is_empty():
			has_target = false
			if is_instance_valid(sphere_inst):
				sphere_inst.visible = false
			if is_instance_valid(line_mesh):
				line_mesh.clear_surfaces()
			return
		else:
			current_target_pos = current_path[0]
			direction = (current_target_pos - drone.global_position)

	# Deterministic 3D Rotation (+Z aligned to current path segment)
	if direction.length() > 0.001:
		var forward = direction.normalized()
		var temp_up = Vector3.UP
		
		if abs(forward.y) > 0.99:
			temp_up = Vector3.FORWARD
		
		var right = temp_up.cross(forward).normalized()
		var up = forward.cross(right).normalized()
		
		drone.global_transform.basis = Basis(right, up, forward)

	# Move toward the active segment target
	drone.global_position = drone.global_position.move_toward(current_target_pos, flight_speed * delta)

func draw_trajectory_line() -> void:
	if not is_instance_valid(line_mesh) or not has_target or current_path.is_empty():
		return
	
	line_mesh.clear_surfaces()
	line_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	
	# Draw line segment from drone to immediate waypoint
	line_mesh.surface_add_vertex(drone.global_position)
	line_mesh.surface_add_vertex(current_path[0])
	
	# Draw the remaining segments of the path
	for i in range(current_path.size() - 1):
		line_mesh.surface_add_vertex(current_path[i])
		line_mesh.surface_add_vertex(current_path[i + 1])
		
	line_mesh.surface_end()

func clamped(val: int, min_val: int, max_val: int) -> int:
	return max(min_val, min(val, max_val))
