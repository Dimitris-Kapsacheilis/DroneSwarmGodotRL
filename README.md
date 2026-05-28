# Drone Swarm

Drone Swarm is a Godot 4.3 prototype for flying a small group of physics-based drones through a simple 3D city scene. It supports manual leader control, follower formations, waypoint assignment, a free-fly camera, and a Godot RL Agents training path.

## Project Layout

- `project.godot` - Godot project settings and main scene entry.
- `scenes/swarm_test.tscn` - Main playable scene.
- `scenes/drone.tscn` - Drone prefab.
- `scripts/swarm_controller.gd` - Spawns drones and manages leaders, formations, waypoint mode, and input actions.
- `scripts/drone.gd` - Drone physics, waypoint seeking, formation following, boids behavior, keyboard control, and RL action hooks.
- `scripts/camera_3d.gd` - Follow camera and manual camera mode.
- `scripts/rl_environment.gd` - Reset, action, observation, reward, and episode helpers for RL.
- `scripts/agents/drone_swarm_ai_controller.gd` - Godot RL Agents-compatible central controller for training the whole swarm.
- `training/train_sb3.py` - Stable-Baselines3 PPO training entrypoint.
- `training/requirements.txt` - Python training dependencies.
- `assets/models/` - Drone and city/map model source files.
- `assets/textures/` - Texture assets.
- `docs/rl_interface.md` - RL observation/action/reward notes.
- `docs/training_setup.md` - Step-by-step Godot RL Agents setup.

## Requirements

- Godot 4.3 or newer.
- Godot RL Agents plugin installed and enabled for training.
- Python 3.10+ with the packages in `training/requirements.txt`.
- Git LFS for large binary assets before committing shared changes:

```powershell
git lfs install
```

## Running

Open the folder in Godot and run the project. The main scene is `res://scenes/swarm_test.tscn`.

## Controls

- `W/S` - Forward/backward thrust for the controlled drone or manual camera.
- `A/D` - Left/right thrust for the controlled drone or manual camera.
- `Space/Ctrl` - Up/down thrust.
- `Q/E` - Yaw left/right.
- `C` - Toggle camera follow/free-fly mode.
- `I` - Toggle swarm mode and individual drone mode.
- `Tab` - Toggle waypoint assignment mode.
- `1-6` - Select swarm leader or individual drone.
- `F1-F5` - Assign the current leader/selected drone to waypoint 1-5.
- `7/8/9/0/-/=` - Select line, V, circle, grid, diamond, or boids formation.

## Current Status

The project is playable as a swarm simulation prototype and has a first proper training path. The recommended starting setup is centralized PPO: one `DroneSwarmAIController` controls all drones through a single continuous action vector. Install the Godot RL Agents plugin, add a `Sync` node to the scene, then run `training/train_sb3.py`.

See `docs/training_setup.md` for the full workflow.

## Asset Notes

Large models and textures are tracked with Git LFS patterns in `.gitattributes`. Asset source/license details should be added before publishing or redistributing this project.
