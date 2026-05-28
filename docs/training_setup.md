# Training Setup

This project is set up for Godot RL Agents with Stable-Baselines3 PPO.

## Recommendation

Start with centralized swarm control:

- One `DroneSwarmAIController` is registered as the Godot RL Agents agent.
- The action vector has `num_drones * 4` continuous values.
- Each drone receives `[strafe, lift, forward, yaw]`.
- The reward is shared and comes from `DroneSwarmRLEnvironment`.

This is the best first step because it avoids multi-agent credit assignment while you tune physics, observations, rewards, and episode resets. After this learns basic waypoint flight, split into one AI controller per drone and move to RLlib multi-agent training.

## Step-by-Step Layout

1. Install the Godot RL Agents plugin.

   Use the Godot Asset Library or the plugin repository:

   - https://godotengine.org/asset-library/asset/1629
   - https://github.com/edbeeching/godot_rl_agents_plugin

2. Enable the plugin in Godot.

   Open `Project > Project Settings > Plugins` and enable Godot RL Agents.

3. Add a `Sync` node to `scenes/swarm_test.tscn`.

   Place it as a direct child of the scene root. Set:

   - `control_mode`: `TRAINING`
   - `action_repeat`: `4` to `8`
   - `speed_up`: `4` to start

   The scene already includes `Drone Swarm AI Controller`, which joins the `AGENT` group and implements the methods the `Sync` node expects.

4. Create a Python virtual environment.

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r training\requirements.txt
```

5. Run in-editor training first.

   Open the project in Godot, keep the editor open, then run:

```powershell
python training\train_sb3.py --experiment_name drone_swarm_editor --timesteps 200000
```

6. Export for faster repeatable training.

   Export a Windows executable to something like:

```text
training/exports/DroneSwarm/DroneSwarm.exe
```

   Then train headless or with visualization:

```powershell
python training\train_sb3.py --env_path training\exports\DroneSwarm\DroneSwarm.exe --experiment_name drone_swarm_export --timesteps 1000000 --speedup 8
```

7. Watch learning in TensorBoard.

```powershell
tensorboard --logdir training\logs\sb3
```

8. Export ONNX for inference.

   `training/train_sb3.py` exports `training/models/drone_swarm_ppo.onnx` by default after training. To use it in Godot, switch the `Sync` node to ONNX inference and point it at that file. Godot RL Agents currently documents ONNX support as experimental.

## What Changed In The Project

- `scripts/agents/drone_swarm_ai_controller.gd`
  Godot RL Agents-compatible central controller. It implements `get_obs`, `get_reward`, `get_action_space`, `set_action`, `get_done`, `reset`, and `get_info`.

- `scripts/rl_environment.gd`
  Owns episode state, reset logic, reward shaping, normalized observations, and flat action dispatch.

- `training/train_sb3.py`
  PPO trainer using `StableBaselinesGodotEnv`.

- `training/requirements.txt`
  Python dependencies for training and ONNX export.

## Observation

The current flat observation includes:

- Remaining episode time.
- Target waypoint index.
- For each drone:
  - Relative target position.
  - Relative leader position.
  - Linear velocity.
  - Angular velocity.
  - Leader flag.

## Reward

The first reward is intentionally simple:

- Negative distance from leader to target.
- Penalty for followers drifting away from the desired follow distance.
- Bonus for reaching the target.
- Penalty for out-of-bounds episodes.
- Small velocity penalty to discourage unstable thrashing.

Tune this before increasing task complexity.

## Next Milestones

1. Train centralized PPO until the leader reliably reaches the first waypoint.
2. Randomize target waypoint and starting positions.
3. Add obstacle/collision penalties.
4. Add curriculum: first one drone, then two, then four.
5. Convert to multi-agent RLlib only after centralized control is stable.
