extends Node3D

const GRID_SIZE = 10
const TILE_SIZE = 2.0
const TOTAL_VOXELS = GRID_SIZE * GRID_SIZE * GRID_SIZE

var fog_voxels: Array = [] # Flat array of 1000 MeshInstance3Ds

func _ready():
	var box_mesh = BoxMesh.new()
	box_mesh.size = Vector3(TILE_SIZE, TILE_SIZE, TILE_SIZE)
	
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.1, 0.1, 0.2, 0.6) # Dark volumetric fog
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED # See through it easily

	for i in range(TOTAL_VOXELS):
		var mi = MeshInstance3D.new()
		mi.mesh = box_mesh
		mi.material_override = mat
		
		# Calculate 3D position from 1D index
		var pos_3d = get_3d_pos_from_index(i)
		mi.position = Vector3(
			pos_3d.x * TILE_SIZE + TILE_SIZE / 2.0, 
			pos_3d.y * TILE_SIZE + TILE_SIZE / 2.0, 
			pos_3d.z * TILE_SIZE + TILE_SIZE / 2.0
		)
		
		add_child(mi)
		fog_voxels.append(mi)

# Maps 1D Action Index (0-999) to 3D Grid Coordinates
func get_3d_pos_from_index(idx: int) -> Vector3i:
	var x = idx % GRID_SIZE
	var z = (idx / GRID_SIZE) % GRID_SIZE
	var y = idx / (GRID_SIZE * GRID_SIZE) # Y is UP
	return Vector3i(x, y, z)

func reveal_voxel(idx: int):
	if idx >= 0 and idx < TOTAL_VOXELS:
		fog_voxels[idx].visible = false

func hide_voxel(idx: int):
	if idx >= 0 and idx < TOTAL_VOXELS:
		fog_voxels[idx].visible = true
