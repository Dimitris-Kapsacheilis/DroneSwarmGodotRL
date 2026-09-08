import argparse
import pathlib
import time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.utils import check_shape_equal

from godot_rl.wrappers.stable_baselines_wrapper import StableBaselinesGodotEnv


def parse_args():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--env_path", default=None, type=str, help="Path to Godot binary (omit for in-editor)")
    parser.add_argument("--model_path", required=True, type=str, help="Path to .zip model file")
    parser.add_argument("--seed", default=42, type=int)
    parser.add_argument("--fps", default=60, type=int, help="Target display FPS")
    parser.add_argument("--speedup", default=1, type=int, help="Engine physics speedup factor")
    parser.add_argument("--max_episode_steps", default=1000, type=int, help="Force reset after N steps")
    parser.add_argument("--deterministic", action="store_true", default=True, help="Use deterministic policy actions")
    return parser.parse_args()


def load_vec_normalize_safely(env, model_zip_path: pathlib.Path):
    """
    Attempts to load VecNormalize statistics. If dimensions do not match
    (e.g., Godot observation space changed), it gracefully falls back without crashing.
    """
    norm_candidates = [
        model_zip_path.parent / f"{model_zip_path.stem}_vec_normalize.pkl",
        model_zip_path.parent / "best_model_vec_normalize.pkl",
        model_zip_path.parent / "final_model_vec_normalize.pkl",
        model_zip_path.parent / "best_vec_normalize.pkl",
    ]

    norm_path = next((p for p in norm_candidates if p.exists()), None)

    if norm_path is None:
        print("[INFO] No VecNormalize statistics file found. Running with unnormalized observations.")
        return env

    try:
        # Attempt standard load
        loaded_env = VecNormalize.load(str(norm_path), env)
        loaded_env.training = False
        loaded_env.norm_reward = False
        print(f"[INFO] Successfully loaded VecNormalize statistics from: {norm_path}")
        return loaded_env
    except AssertionError as e:
        print(f"\n[WARNING] Observation dimension mismatch in {norm_path.name}:")
        print(f"          {e}")
        print("          This happens when the Godot binary observation size was updated after this checkpoint.")
        print("          -> Falling back to dynamic VecNormalize (inference mode).\n")

        # Create fresh VecNormalize wrapper matching the new environment dimensions
        fallback_env = VecNormalize(env, norm_obs=True, norm_reward=False, training=False)
        return fallback_env
    except Exception as e:
        print(f"[WARNING] Could not load VecNormalize: {e}. Proceeding with base environment.")
        return env


def main():
    args = parse_args()
    model_zip = pathlib.Path(args.model_path).resolve()

    if not model_zip.exists():
        raise FileNotFoundError(f"Model file not found: {model_zip}")

    print(f"[1/3] Launching Godot Environment ({'In-Editor' if args.env_path is None else args.env_path})...")
    raw_env = StableBaselinesGodotEnv(
        env_path=args.env_path,
        show_window=True,
        seed=args.seed,
        n_parallel=1,
        speedup=args.speedup,
    )

    env = VecMonitor(raw_env)

    print("[2/3] Configuring Normalization Wrapper...")
    env = load_vec_normalize_safely(env, model_zip)

    print(f"[3/3] Loading Model weights from: {model_zip.name}...")
    try:
        model = PPO.load(model_zip, env=env)
    except Exception as e:
        print(f"\n[ERROR] Failed to load policy weights: {e}")
        print("Tip: If you changed observation dimensions in Godot, you need to retrain a new model.")
        raw_env.close()
        return

    print("\n=======================================================")
    print("  Swarm Inference Running (Press Ctrl+C to stop)       ")
    print("=======================================================\n")

    target_frame_duration = 1.0 / max(1, args.fps)
    step_count = 0
    obs = env.reset()

    try:
        while True:
            start_time = time.time()

            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, rewards, dones, infos = env.step(action)
            step_count += 1

            # Check if any sub-environment reached terminal state
            is_done = dones[0] if isinstance(dones, (list, np.ndarray)) else dones
            if is_done or step_count >= args.max_episode_steps:
                reason = "Natural Terminal State" if is_done else f"Max Steps Reached ({args.max_episode_steps})"
                print(f"[Reset] Episode finished: {reason} at step {step_count}")
                obs = env.reset()
                step_count = 0

            # Frame rate throttle for smooth visualization
            elapsed = time.time() - start_time
            sleep_time = target_frame_duration - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInference stopped by user.")
    finally:
        print("Closing environment...")
        env.close()


if __name__ == "__main__":
    main()