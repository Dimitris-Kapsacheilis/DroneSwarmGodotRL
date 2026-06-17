# RL Interface

The Godot-side RL API lives in `res://scripts/rl_environment.gd` and is attached to `RL Environment` in `res://scenes/swarm_test.tscn`. Godot RL Agents integration is handled by `res://scripts/agents/drone_swarm_ai_controller.gd`.

## Episode API

- `reset(random_seed := 0) -> Dictionary`
  Resets drone positions, rebuilds mission state, clears coverage/fog/target memory, and returns the first step result.
- `apply_actions(actions: Array) -> void`
  Helper for assigning task indices directly.
- `apply_flat_action(action_values: Array) -> void`
  Applies the flat Godot RL Agents task-score vector from the centralized controller.
- `step(actions: Array) -> Dictionary`
  Applies actions and returns the current observation, reward, done flag, episode id, and elapsed time. External trainers should call this once per physics frame or after awaiting `physics_frame`.
- `collect_step_result() -> Dictionary`
  Reads the current environment state without changing actions.
- `get_telemetry() -> Array[Dictionary]`
  Returns compact per-step telemetry for reward, elapsed time, target, done state, and drone count.
- `clear_telemetry() -> void`
  Clears accumulated telemetry.

## Action Space

Godot RL Agents sees one centralized continuous action named `task_assignments`.
Its size is:

```text
num_drones * mission_task_count
```

For each drone, the policy outputs one score per mission task. The environment
uses the highest-scoring task as that drone's assignment. Tasks are generated from:

- Area-coverage cells.
- Moving target-follow tasks.
- A rally task.

The drone does not learn micro-control. Once assigned, it flies with the built-in
`Drone.go_to_waypoint()` autopilot.

Direct task-index dictionaries are also accepted by the environment helper:

```gdscript
{
	"task": 3
}
```

## Observation Space

The structured observation dictionary contains:

- `mission` - Coverage, target-known, target-tracked, and task-count summary.
- `coverage_cells` - Fog-of-war coverage cells with center, covered flag, recent visibility, blocked/no-fly flag, and assignment count.
- `targets` - Sanitized target memory. True target positions are only exposed while visible.
- `assignments` - Current task index per drone.
- `drones` - One dictionary per drone with position, velocity, assignment, task waypoint, and no-fly-zone state.

For Godot RL Agents training, `get_flat_observation()` returns a fixed-size
numeric array for Stable-Baselines3. It includes:

- Remaining time.
- Coverage fraction.
- Known target fraction.
- Tracked target fraction.
- No-fly-zone violation fraction.
- Per-drone relative position, velocity, assigned-task vector, nearest no-fly-zone vector, assignment index, altitude, and violation flag.
- Per-cell relative position, covered flag, recent visibility flag, blocked flag, and assignment count.
- Per-target last-known/search position, known flag, memory age, tracked flag, and assignment count.

## Reward

The reward targets task allocation and distribution:

- Bonus for newly covered area cells.
- Dense reward for increasing total coverage.
- Bonus for detecting hidden/moving targets.
- Per-step reward for keeping targets tracked.
- Penalty when known target memory goes stale.
- Penalty for duplicate assignment concentration.
- Penalty for excessive assignment churn.
- Penalty for no-fly-zone and altitude violations.
- Completion bonus when coverage is complete and all targets are tracked.

It also penalizes out-of-bounds episodes.

## Done Conditions

An episode ends when:

- Coverage is complete and all targets are tracked.
- `episode_seconds` is exceeded.
- Any drone moves farther than `max_distance_from_origin`.

## Scene Integration

`res://scenes/drone.tscn` no longer registers its own `AIController3D` child. The
single Godot RL Agents agent is `Drone Swarm AI Controller` in
`res://scenes/swarm_test.tscn`, matching the custom-environment pattern from
Godot RL Agents while keeping credit assignment centralized.
