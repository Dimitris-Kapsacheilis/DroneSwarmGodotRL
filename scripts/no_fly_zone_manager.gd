extends Node3D

@export var num_zones: int = 3
@export var map_area: Rect2 = Rect2(0, 0, 30, 30)
@export var min_zone_size: float = 3.0
@export var max_zone_size: float = 12.0
@export var min_height: float = 0.0
@export var max_height: float = 30.0

@export var drone_clearance: float = 4.0

var zones: Array[NoFlyZone] = []

func _ready() -> void:
	randomize()
	generate_random_zones()

func reset_nfz() -> void:
	clear_zones()
	generate_random_zones()

func clear_zones() -> void:
	for zone in zones:
		if is_instance_valid(zone):
			if zone.get_parent():
				zone.get_parent().remove_child(zone)
			zone.queue_free()
	zones.clear()

func generate_random_zones() -> void:
	var placed_rects: Array[Rect2] = []
	var max_attempts := 150

	for i in range(num_zones):
		var zone_placed := false
		
		for attempt in range(max_attempts):
			var width := randf_range(min_zone_size, max_zone_size)
			var height := randf_range(min_zone_size, max_zone_size)
			
			var min_x := map_area.position.x + drone_clearance
			var min_y := map_area.position.y + drone_clearance
			var max_x := map_area.end.x - drone_clearance - width
			var max_y := map_area.end.y - drone_clearance - height
			
			if max_x < min_x or max_y < min_y:
				continue
				
			var pos_x := randf_range(min_x, max_x)
			var pos_y := randf_range(min_y, max_y)
			
			var candidate_rect := Rect2(pos_x, pos_y, width, height)
			var clearance_rect := candidate_rect.grow(drone_clearance)
			var overlaps := false
			
			for existing_rect in placed_rects:
				if clearance_rect.intersects(existing_rect):
					overlaps = true
					break
			
			if not overlaps:
				placed_rects.append(candidate_rect)
				instantiate_zone(candidate_rect)
				zone_placed = true
				break
		
		if not zone_placed:
			push_warning("Could not place zone %d without blocking drone clearance." % [i + 1])

func instantiate_zone(rect: Rect2) -> void:
	var zone = NoFlyZone.new()
	add_child(zone)
	
	var points: Array[Vector2] = [
		rect.position,
		Vector2(rect.end.x, rect.position.y),
		rect.end,
		Vector2(rect.position.x, rect.end.y)
	]
	
	zone.setup(points, min_height, max_height)
	zones.append(zone)

# Marks all cells inside NFZs as blocked in GridManager
func populate_blocked_cells(grid_manager: Node3D) -> void:
	if not is_instance_valid(grid_manager):
		return

	var grid_size: Vector3i = grid_manager.grid_size
	for zone in zones:
		if not is_instance_valid(zone) or zone.is_queued_for_deletion():
			continue

		var points = zone.polygon
		if points.is_empty():
			continue

		var min_x = points[0].x; var max_x = points[0].x
		var min_z = points[0].y; var max_z = points[0].y
		for p in points:
			min_x = minf(min_x, p.x); max_x = maxf(max_x, p.x)
			min_z = minf(min_z, p.y); max_z = maxf(max_z, p.y)

		var min_coord = grid_manager.world_to_grid(Vector3(min_x, zone.min_altitude, min_z))
		var max_coord = grid_manager.world_to_grid(Vector3(max_x, zone.max_altitude, max_z))

		for x in range(clampi(min_coord.x, 0, grid_size.x - 1), clampi(max_coord.x + 1, 0, grid_size.x)):
			for y in range(clampi(min_coord.y, 0, grid_size.y - 1), clampi(max_coord.y + 1, 0, grid_size.y)):
				for z in range(clampi(min_coord.z, 0, grid_size.z - 1), clampi(max_coord.z + 1, 0, grid_size.z)):
					var coord = Vector3i(x, y, z)
					var world_p = grid_manager.grid_to_world(coord)
					if Geometry2D.is_point_in_polygon(Vector2(world_p.x, world_p.z), points):
						grid_manager.blocked_cells[coord] = true
