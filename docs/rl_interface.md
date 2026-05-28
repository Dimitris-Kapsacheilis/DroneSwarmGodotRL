# RL Interface

The Godot-side RL API lives in `res://scripts/rl_environment.gd` and is attached to `RL Environment` in `res://scenes/swarm_test.tscn`. Godot RL Agents integration is handled by `res://scripts/agents/drone_swarm_ai_controller.gd`.

## Episode API

- `reset(random_seed := 0) -> Dictionary`
  Resets drone positions, clears control state, resets the swarm controller, and returns the first step result.
- `apply_actions(actions: Array) -> void`
  Applies one normalized action per drone.
- `apply_flat_action(action_values: Array) -> void`
  Applies one flat `num_drones * 4` action vector from the centralized Godot RL Agents controller.
- `step(actions: Array) -> Dictionary`
  Applies actions and returns the current observation, reward, done flag, episode id, and elapsed time. External trainers should call this once per physics frame or after awaiting `physics_frame`.
- `collect_step_result() -> Dictionary`
  Reads the current environment state without changing actions.
- `get_telemetry() -> Array[Dictionary]`
  Returns compact per-step telemetry for reward, elapsed time, target, done state, and drone count.
- `clear_telemetry() -> void`
  Clears accumulated telemetry.

## Action Space

Each drone action is four normalized values in `[-1, 1]`:

```text
[strafe, lift, forward, yaw]
```

Dictionaries are also accepted:

```gdscript
{
	"strafe": 0.0,
	"lift": 0.4,
	"forward": 1.0,
	"yaw": -0.2
}
```

## Observation Space

The structured observation dictionary contains:

- `target` - The active waypoint as `[x, y, z]`.
- `formation` - Current formation name.
- `drones` - One dictionary per drone with id, position, rotation, linear velocity, angular velocity, leader flag, and waypoint state.

For Godot RL Agents training, `get_flat_observation()` returns a fixed-size numeric array for Stable-Baselines3.

## Reward

The default reward favors:

- Reducing leader distance to the target waypoint.
- Keeping followers near the configured follow distance.
- Reaching the target within `success_radius`.

It penalizes out-of-bounds episodes. Treat this as a baseline reward, not a finished research objective.

## Done Conditions

An episode ends when:

- The leader reaches the target waypoint.
- `episode_seconds` is exceeded.
- Any drone moves farther than `max_distance_from_origin`.

## Next Training Step

Add a transport layer between Godot and Python, such as a TCP/WebSocket bridge, Godot RL Agents, or a custom headless runner. The existing API is shaped so that bridge only has to call `reset`, `apply_actions`, and `collect_step_result`.
