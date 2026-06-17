extends Node3D # Changed from CharacterBody3D to Node3D since this is a child node

@onready var grid_manager =  get_node_or_null("/root/Swarm Test/GridManager") # Drag your GridManager node here in the Inspector
@export var flight_speed: float = 10.0
@export var arrival_threshold: float = 0.5

@onready var drone: Node3D = get_parent() # Automatically gets the parent drone node

var current_target_pos: Vector3 = Vector3.ZERO
var has_target: bool = false

# Visualizer variables
var sphere_inst: MeshInstance3D
var line_inst: MeshInstance3D
var line_mesh: ImmediateMesh

func _ready() -> void:
	randomize()
	# Wait one frame to ensure the scene tree is ready, then setup visualizers
	await get_tree().process_frame
	setup_visualizers()

func _physics_process(delta: float) -> void:
	if not is_instance_valid(grid_manager) or not is_instance_valid(drone):
		return

	if not has_target:
		choose_new_random_target()

	fly_toward_target(delta)
	draw_trajectory_line()

func setup_visualizers() -> void:
	if not is_instance_valid(drone):
		return
		
	var world_root = drone.get_parent()
	if not world_root:
		return

	# 1. Setup Waypoint Sphere (Green)
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
	
	# Add to the world root so it stays stationary in world space
	world_root.add_child(sphere_inst)

	# 2. Setup Trajectory Line (Red)
	var line_material = StandardMaterial3D.new()
	line_material.albedo_color = Color.RED
	line_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	line_mesh = ImmediateMesh.new()
	line_inst = MeshInstance3D.new()
	line_inst.mesh = line_mesh
	line_inst.material_override = line_material
	
	# Add to the world root to draw in world-space coordinates
	world_root.add_child(line_inst)

func choose_new_random_target() -> void:
	var bounds = grid_manager.grid_size
	
	# Pick a random integer coordinate strictly within the grid boundaries (0 to bounds - 1)
	var random_coord = Vector3i(
		randi_range(0, bounds.x - 1),
		randi_range(0, bounds.y - 1),
		randi_range(0, bounds.z - 1)
	)
	
	# Convert grid coordinate to 3D world position (centered in the 1x1x1 box)
	var local_target = Vector3(random_coord.x + 0.5, random_coord.y + 0.5, random_coord.z + 0.5)
	current_target_pos = grid_manager.to_global(local_target)
	
	has_target = true

	# Move the green sphere to the new target position and make it visible
	if is_instance_valid(sphere_inst):
		sphere_inst.global_position = current_target_pos
		sphere_inst.visible = true

func fly_toward_target(delta: float) -> void:
	# Reference the drone's global_position instead of self
	var direction = (current_target_pos - drone.global_position)
	var distance = direction.length()

	# Check if the drone has arrived at the target box
	if distance <= arrival_threshold:
		has_target = false
		if is_instance_valid(sphere_inst):
			sphere_inst.visible = false
		if is_instance_valid(line_mesh):
			line_mesh.clear_surfaces()
		return

	# Rotate drone smoothly to face the target destination
	if direction.length() > 0.1:
		var target_look = drone.global_position - direction.normalized()
		var target_transform = drone.global_transform.looking_at(target_look, Vector3.UP)
		drone.global_transform = drone.global_transform.interpolate_with(target_transform, 5.0 * delta)

	# Move parent drone toward target
	drone.global_position = drone.global_position.move_toward(current_target_pos, flight_speed * delta)

func draw_trajectory_line() -> void:
	if not is_instance_valid(line_mesh) or not has_target or not is_instance_valid(drone):
		return
	
	# Redraw the red line from the parent drone's current position to the static waypoint
	line_mesh.clear_surfaces()
	line_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	line_mesh.surface_add_vertex(drone.global_position)
	line_mesh.surface_add_vertex(current_target_pos)
	line_mesh.surface_end()
