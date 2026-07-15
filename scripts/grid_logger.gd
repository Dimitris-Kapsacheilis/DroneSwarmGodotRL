class_name GridLogger
extends Node

@export var enable_logging: bool = true
@export var save_every_n_episodes: int = 10

var episode_count: int = 0
var episode_visited: Array[Vector3i] = []     # Positions visited this episode
var total_visits: Dictionary = {}              # Cumulative: "x,y,z" -> visit count

var session_id: String = ""
var session_dir: String = ""

func _ready() -> void:
	if enable_logging:
		_initialize_session()

func _initialize_session() -> void:
	session_id = Time.get_datetime_string_from_system().replace(":", "-").replace("T", "_")
	session_dir = "user://Grid_HeatMap/session_" + session_id + "/"
	_create_directory_structure()

func _create_directory_structure() -> void:
	var dir = DirAccess.open("user://")
	if not dir:
		push_error("GridLogger: Unable to open 'user://' directory.")
		return
	
	# Helper to ensure directories exist safely
	var paths_to_create = [
		"Grid_HeatMap",
		"Grid_HeatMap/session_" + session_id,
		"Grid_HeatMap/session_" + session_id + "/json",
		"Grid_HeatMap/session_" + session_id + "/csv",
		"Grid_HeatMap/session_" + session_id + "/cumulative"
	]
	
	for path in paths_to_create:
		if not dir.dir_exists(path):
			var err = dir.make_dir(path)
			if err != OK:
				push_warning("GridLogger: Failed to create directory: %s. Error code: %d" % [path, err])

func log_visited(coord: Vector3i) -> void:
	if not enable_logging:
		return
	
	if not episode_visited.has(coord):
		episode_visited.append(coord)
	
	var key = "%d,%d,%d" % [coord.x, coord.y, coord.z]
	if not total_visits.has(key):
		total_visits[key] = 0
	total_visits[key] += 1

func clear_episode_data() -> void:
	episode_visited.clear()

func save_episode_data(coverage_percent: float, unique_visited_count: int) -> void:
	if not enable_logging or episode_visited.is_empty():
		return
	
	episode_count += 1
	
	# Session directory fallback check
	if session_dir.is_empty():
		_initialize_session()
	
	var data = {
		"episode": episode_count,
		"visited_count": episode_visited.size(),
		"coverage_percent": coverage_percent,
		"visited_nodes": []
	}
	
	for pos in episode_visited:
		data["visited_nodes"].append({"x": pos.x, "y": pos.y, "z": pos.z})
	
	# Save JSON (organized inside /json subfolder)
	var json_path = session_dir + "json/episode_%04d.json" % episode_count
	var json_file = FileAccess.open(json_path, FileAccess.WRITE)
	if json_file:
		json_file.store_string(JSON.stringify(data, "\t"))
		json_file.close()
	else:
		push_error("GridLogger: Failed to save JSON path: " + json_path)
	
	# Save CSV (organized inside /csv subfolder)
	var csv_path = session_dir + "csv/episode_%04d.csv" % episode_count
	var csv_file = FileAccess.open(csv_path, FileAccess.WRITE)
	if csv_file:
		csv_file.store_line("x,y,z")
		for pos in episode_visited:
			csv_file.store_line("%d,%d,%d" % [pos.x, pos.y, pos.z])
		csv_file.close()
	else:
		push_error("GridLogger: Failed to save CSV path: " + csv_path)
	
	# Save cumulative data occasionally
	if episode_count % save_every_n_episodes == 0:
		_save_cumulative(unique_visited_count)

func _save_cumulative(unique_visited_count: int) -> void:
	if session_dir.is_empty():
		return
		
	var data = {
		"total_episodes": episode_count,
		"total_unique_visited": unique_visited_count,
		"visits": []
	}
	for key in total_visits:
		var parts = key.split(",")
		data["visits"].append({
			"x": int(parts[0]),
			"y": int(parts[1]),
			"z": int(parts[2]),
			"count": total_visits[key]
		})
	
	# Save Cumulative JSON (organized inside /cumulative subfolder)
	var cumulative_path = session_dir + "cumulative/drone_total_visits.json"
	var file = FileAccess.open(cumulative_path, FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify(data, "\t"))
		file.close()
		print("📊 Saved cumulative visits up to episode %d to: %s" % [episode_count, cumulative_path])
	else:
		push_error("GridLogger: Failed to save cumulative file: " + cumulative_path)
