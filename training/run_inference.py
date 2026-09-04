import argparse
import pathlib
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize

from godot_rl.wrappers.stable_baselines_wrapper import StableBaselinesGodotEnv

parser = argparse.ArgumentParser(allow_abbrev=False)
parser.add_argument("--env_path", required=True, type=str, help="Path to Godot binary")
parser.add_argument("--model_path", required=True, type=str, help="Path to best_model.zip")
parser.add_argument("--seed", default=42, type=int)
parser.add_argument("--fps", default=60, type=int)
parser.add_argument("--max_episode_steps", default=1000, type=int, help="Force reset after N steps if stuck")
args = parser.parse_args()

model_zip = pathlib.Path(args.model_path)
if not model_zip.exists():
    raise FileNotFoundError(f"Model file not found: {model_zip}")

# 1. Initialize Base Godot Environment
env = StableBaselinesGodotEnv(
    env_path=args.env_path,
    show_window=True,
    seed=args.seed,
    n_parallel=1,
    speedup=1,
)

env = VecMonitor(env)

# 2. Find and Load VecNormalize
norm_candidates = [
    model_zip.parent / f"{model_zip.stem}_vec_normalize.pkl",
    model_zip.parent / "best_model_vec_normalize.pkl",
    model_zip.parent / "best_vec_normalize.pkl",
]

norm_path = next((str(p) for p in norm_candidates if p.exists()), None)
if norm_path:
    print(f"[INFO] Loaded Normalization statistics from: {norm_path}")
    env = VecNormalize.load(norm_path, env)
    env.training = False
    env.norm_reward = False
else:
    print("[WARNING] No VecNormalize statistics found! Drones might act unstable.")

# 3. Load Model
model = PPO.load(model_zip, env=env)
print("\n=== Running Inference (Press Ctrl+C to stop) ===")

target_frame_duration = 1.0 / args.fps
step_count = 0
obs = env.reset()

try:
    while True:
        start_time = time.time()

        action, _ = model.predict(obs, deterministic=True)
        obs, rewards, dones, infos = env.step(action)
        step_count += 1

        # Check if environment finished naturally OR if max steps reached
        if dones[0] or step_count >= args.max_episode_steps:
            reason = "Natural Terminal State" if dones[0] else f"Reached Max Steps ({args.max_episode_steps})"
            print(f"[Reset] Resetting environment: {reason}")
            obs = env.reset()
            step_count = 0

        # Maintain smooth visual FPS
        elapsed = time.time() - start_time
        sleep_time = target_frame_duration - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    env.close()