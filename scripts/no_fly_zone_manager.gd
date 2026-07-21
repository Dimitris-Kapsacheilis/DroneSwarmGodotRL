extends Node3D

# Configuration options editable in the Inspector
@export var num_zones: int = 3
@export var map_area: Rect2 = Rect2(0, 0, 30, 30) # Bounds for generation
@export var min_zone_size: float = 3.0
@export var max_zone_size: float = 12.0
@export var min_height: float = 0.0
@export var max_height: float = 30.0

# The clearance width required for the drone to pass through (e.g., 1.0 for a 1x1x1 drone)
@export var drone_clearance: float = 4.0

var zones: Array[NoFlyZone] = []

func _ready() -> void:
	generate_random_zones()

func generate_random_zones() -> void:
	var placed_rects: Array[Rect2] = []
	var max_attempts := 150 # Increased attempts to help find valid placements with clearance

	for i in range(num_zones):
		var zone_placed := false
		
		for attempt in range(max_attempts):
			# Determine random dimensions for the zone
			var width := randf_range(min_zone_size, max_zone_size)
			var height := randf_range(min_zone_size, max_zone_size)
			
			# Enforce boundary limits, keeping the zone at least 'drone_clearance' away from the map borders
			var min_x := map_area.position.x + drone_clearance
			var min_y := map_area.position.y + drone_clearance
			var max_x := map_area.end.x - drone_clearance - width
			var max_y := map_area.end.y - drone_clearance - height
			
			# If the remaining space is too small for this zone size, try again
			if max_x < min_x or max_y < min_y:
				continue
				
			var pos_x := randf_range(min_x, max_x)
			var pos_y := randf_range(min_y, max_y)
			
			var candidate_rect := Rect2(pos_x, pos_y, width, height)
			
			# Check if this zone is too close to any existing zones.
			# We expand the candidate rect by 'drone_clearance' to ensure sufficient spacing.
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
			push_warning("Could not place zone %d without blocking drone clearance after %d attempts." % [i + 1, max_attempts])

func instantiate_zone(rect: Rect2) -> void:
	var zone = NoFlyZone.new()
	add_child(zone)
	
	# Generate the 4 corners of the Rect2 in sequence
	var points: Array[Vector2] = [
		rect.position,                               # Top-Left
		Vector2(rect.end.x, rect.position.y),       # Top-Right
		rect.end,                                    # Bottom-Right
		Vector2(rect.position.x, rect.end.y)        # Bottom-Left
	]
	
	zone.setup(points, min_height, max_height)
	zones.append(zone)
