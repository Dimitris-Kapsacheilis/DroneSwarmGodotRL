import argparse
import os
from pathlib import Path
from typing import Callable

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor

from godot_rl.wrappers.onnx.stable_baselines_export import export_model_as_onnx
from godot_rl.wrappers.stable_baselines_wrapper import StableBaselinesGodotEnv


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    def schedule(progress_remaining: float) -> float:
        return progress_remaining * initial_value

    return schedule


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--env_path", default=None, type=str, help="Exported Godot executable. Omit for in-editor training.")
    parser.add_argument("--experiment_dir", default="training/logs/sb3", type=str)
    parser.add_argument("--experiment_name", default="drone_swarm_ppo", type=str)
    parser.add_argument("--seed", default=1, type=int)
    parser.add_argument("--timesteps", default=1_000_000, type=int)
    parser.add_argument("--speedup", default=4, type=int)
    parser.add_argument("--n_parallel", default=1, type=int)
    parser.add_argument("--viz", action="store_true", help="Show the Godot window when using --env_path.")
    parser.add_argument("--resume_model_path", default=None, type=str)
    parser.add_argument("--save_model_path", default="training/models/drone_swarm_ppo.zip", type=str)
    parser.add_argument("--save_checkpoint_frequency", default=100_000, type=int)
    parser.add_argument("--onnx_export_path", default="training/models/drone_swarm_ppo.onnx", type=str)
    parser.add_argument("--inference", action="store_true")
    parser.add_argument("--linear_lr_schedule", action="store_true")
    return parser


def main() -> None:
    args, _extras = build_parser().parse_known_args()

    if args.inference and args.resume_model_path is None:
        raise ValueError("--inference requires --resume_model_path")

    Path(args.experiment_dir).mkdir(parents=True, exist_ok=True)
    if args.save_model_path:
        Path(args.save_model_path).parent.mkdir(parents=True, exist_ok=True)
    if args.onnx_export_path:
        Path(args.onnx_export_path).parent.mkdir(parents=True, exist_ok=True)

    env = StableBaselinesGodotEnv(
        env_path=args.env_path,
        show_window=args.viz,
        seed=args.seed,
        n_parallel=args.n_parallel,
        speedup=args.speedup,
    )
    env = VecMonitor(env)

    learning_rate = linear_schedule(0.0003) if args.linear_lr_schedule else 0.0003

    if args.resume_model_path:
        print(f"Loading model: {os.path.abspath(args.resume_model_path)}")
        model = PPO.load(args.resume_model_path, env=env, tensorboard_log=args.experiment_dir)
    else:
        model = PPO(
            "MultiInputPolicy",
            env,
            ent_coef=0.0001,
            learning_rate=learning_rate,
            n_steps=256,
            batch_size=256,
            gamma=0.99,
            gae_lambda=0.95,
            verbose=2,
            tensorboard_log=args.experiment_dir,
        )

    callbacks = []
    if args.save_checkpoint_frequency:
        checkpoint_dir = Path(args.experiment_dir) / f"{args.experiment_name}_checkpoints"
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        callbacks.append(
            CheckpointCallback(
                save_freq=max(args.save_checkpoint_frequency // max(env.num_envs, 1), 1),
                save_path=str(checkpoint_dir),
                name_prefix=args.experiment_name,
            )
        )

    try:
        if args.inference:
            obs = env.reset()
            for _ in range(args.timesteps):
                action, _state = model.predict(obs, deterministic=True)
                obs, _reward, _done, _info = env.step(action)
        else:
            model.learn(
                total_timesteps=args.timesteps,
                tb_log_name=args.experiment_name,
                callback=callbacks or None,
            )
    finally:
        if args.save_model_path and not args.inference:
            print(f"Saving model: {os.path.abspath(args.save_model_path)}")
            model.save(Path(args.save_model_path).with_suffix(".zip"))

        if args.onnx_export_path and not args.inference:
            onnx_path = Path(args.onnx_export_path).with_suffix(".onnx")
            print(f"Exporting ONNX: {os.path.abspath(onnx_path)}")
            export_model_as_onnx(model, str(onnx_path))

        env.close()


if __name__ == "__main__":
    main()
