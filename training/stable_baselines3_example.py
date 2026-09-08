import argparse
import datetime
import pathlib
from typing import Callable

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
)
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize

from godot_rl.core.utils import can_import
from godot_rl.wrappers.onnx.stable_baselines_export import export_model_as_onnx
from godot_rl.wrappers.stable_baselines_wrapper import StableBaselinesGodotEnv


if can_import("ray"):
    print("WARNING, stable baselines and ray[rllib] are not compatible")


parser = argparse.ArgumentParser(allow_abbrev=False)
parser.add_argument(
    "--env_path",
    default=None,
    type=str,
    help="The Godot binary to use, do not include for in-editor training",
)
parser.add_argument(
    "--experiment_dir",
    default="./logs",
    type=str,
    help="Base directory to store run logs, models, and checkpoints",
)
parser.add_argument(
    "--experiment_name",
    default="drone_swarm_ppo",
    type=str,
    help="Name of the experiment",
)
parser.add_argument("--seed", type=int, default=42, help="Seed of the experiment")
parser.add_argument(
    "--resume_model_path",
    default=None,
    type=str,
    help="Path to a model file (.zip) to resume training or run inference.",
)
parser.add_argument(
    "--save_model_path",
    default=None,
    type=str,
    help="Path to save the final trained SB3 model (.zip)",
)
parser.add_argument(
    "--save_checkpoint_frequency",
    default=200_000,
    type=int,
    help="Save checkpoints every N environment steps",
)
parser.add_argument(
    "--best_model_start_step",
    default=0,
    type=int,
    help="Step count after which best model tracking starts",
)
parser.add_argument(
    "--best_model_check_freq",
    default=50_000,
    type=int,
    help="Frequency (in steps) to check for new best reward",
)
parser.add_argument(
    "--onnx_export_path",
    default=None,
    type=str,
    help="Path to export the ONNX model after training",
)
parser.add_argument(
    "--timesteps",
    default=50_000_000,
    type=int,
    help="Total timesteps to train",
)
parser.add_argument(
    "--inference",
    default=False,
    action="store_true",
    help="Run inference instead of training",
)
parser.add_argument(
    "--no_linear_lr_schedule",
    default=False,
    action="store_true",
    help="Disable linear learning rate decay",
)
parser.add_argument(
    "--viz",
    action="store_true",
    help="Display simulation window during training",
    default=False,
)
parser.add_argument(
    "--speedup",
    default=8,
    type=int,
    help="Physics engine speedup factor (e.g. 4, 8, 16)",
)
parser.add_argument(
    "--action_repeat",
    default=None,
    type=int,
    help="Frame skip count (action repeat)",
)
parser.add_argument(
    "--n_parallel",
    default=1,
    type=int,
    help="Number of parallel Godot executable instances",
)
parser.add_argument("--learning_rate", default=1e-4, type=float, help="Initial learning rate")
parser.add_argument("--n_steps", default=2048, type=int, help="Steps per rollout")
parser.add_argument("--batch_size", default=2048, type=int, help="Minibatch size for PPO")
parser.add_argument("--n_epochs", default=4, type=int, help="Optimization epochs per rollout")
parser.add_argument("--ent_coef", default=0.015, type=float, help="Entropy coefficient")
parser.add_argument("--clip_range", default=0.2, type=float, help="PPO clip range")
parser.add_argument("--gae_lambda", default=0.95, type=float, help="GAE lambda")
parser.add_argument("--gamma", default=0.99, type=float, help="Discount factor")

args, extras = parser.parse_known_args()


class RollingBestModelCallback(BaseCallback):
    def __init__(
        self,
        check_freq_steps: int,
        save_path: str,
        start_step: int = 0,
        min_episodes: int = 5,
        verbose: int = 1,
    ):
        super().__init__(verbose)
        self.check_freq_steps = check_freq_steps
        self.save_path = pathlib.Path(save_path).resolve()
        self.start_step = start_step
        self.min_episodes = min_episodes
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        self.save_path.mkdir(parents=True, exist_ok=True)

    def _on_step(self) -> bool:
        if self.num_timesteps < self.start_step:
            return True

        if self.n_calls % self.check_freq_steps == 0:
            ep_buffer = self.model.ep_info_buffer
            if ep_buffer and len(ep_buffer) >= self.min_episodes:
                mean_reward = float(np.mean([ep_info["r"] for ep_info in ep_buffer]))
                mean_len = float(np.mean([ep_info["l"] for ep_info in ep_buffer]))

                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    self.save_path.mkdir(parents=True, exist_ok=True)

                    model_path = self.save_path / "best_model.zip"
                    self.model.save(str(model_path))

                    vec_norm = self.model.get_vec_normalize_env()
                    if vec_norm is not None:
                        norm_path = self.save_path / "best_model_vec_normalize.pkl"
                        vec_norm.save(str(norm_path))

                    if self.verbose > 0:
                        print(
                            f"\n>>> [BestModel] Step {self.num_timesteps:,} (Buffer: {len(ep_buffer)} eps): "
                            f"New best mean reward: {mean_reward:.2f} (avg len: {mean_len:.1f}). "
                            f"Saved to: {model_path}"
                        )
        return True


def handle_onnx_export(model):
    if args.onnx_export_path is not None and model is not None:
        path_onnx = pathlib.Path(args.onnx_export_path).with_suffix(".onnx").resolve()
        path_onnx.parent.mkdir(parents=True, exist_ok=True)
        print(f"Exporting ONNX to: {path_onnx}")
        export_model_as_onnx(model, str(path_onnx))


def handle_model_save(model, run_dir):
    save_path = pathlib.Path(args.save_model_path) if args.save_model_path else (run_dir / "final_model.zip")
    save_path = save_path.with_suffix(".zip").resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)

    if model is not None:
        print(f"Saving final model to: {save_path}")
        model.save(str(save_path))
        vec_norm = model.get_vec_normalize_env() if hasattr(model, "get_vec_normalize_env") else None
        if vec_norm is not None:
            norm_path = save_path.parent / f"{save_path.stem}_vec_normalize.pkl"
            print(f"Saving VecNormalize stats to: {norm_path}")
            vec_norm.save(str(norm_path))


def close_env(env):
    if env is not None:
        try:
            print("Closing environment...")
            env.close()
        except Exception as e:
            print(f"Exception while closing env: {e}")


def cleanup(model, env, run_dir):
    handle_onnx_export(model)
    handle_model_save(model, run_dir)
    close_env(env)


if args.inference and args.resume_model_path is None:
    raise parser.error("Using --inference requires --resume_model_path to be set.")

timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
run_name = f"{args.experiment_name}_{timestamp}"
run_dir = (pathlib.Path(args.experiment_dir) / run_name).resolve()

path_checkpoint = run_dir / "checkpoints"
path_best_model = run_dir / "best_model"
run_dir.mkdir(parents=True, exist_ok=True)
print(f"Run output directory: {run_dir}")

env = StableBaselinesGodotEnv(
    env_path=args.env_path,
    show_window=args.viz,
    seed=args.seed,
    n_parallel=args.n_parallel,
    speedup=args.speedup,
    action_repeat=args.action_repeat,
)
env = VecMonitor(env)

norm_path = None
if args.resume_model_path:
    resume_p = pathlib.Path(args.resume_model_path).resolve()
    potential_norm = resume_p.parent / f"{resume_p.stem}_vec_normalize.pkl"
    if potential_norm.exists():
        norm_path = str(potential_norm)

if norm_path:
    print(f"Loading existing normalization statistics from {norm_path}")
    env = VecNormalize.load(norm_path, env)
    if args.inference:
        env.training = False
        env.norm_reward = False
else:
    env = VecNormalize(
        env,
        norm_obs=True,
        norm_reward=not args.inference,
        clip_obs=10.0,
        clip_reward=10.0,
        gamma=args.gamma,
    )


def linear_schedule_with_floor(initial_value: float, min_fraction: float = 0.1) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:
        return initial_value * (min_fraction + (1.0 - min_fraction) * progress_remaining)
    return func


use_linear_lr = not args.no_linear_lr_schedule
learning_rate = linear_schedule_with_floor(args.learning_rate, min_fraction=0.1) if use_linear_lr else args.learning_rate

policy_kwargs = dict(
    net_arch=dict(pi=[512, 512], vf=[512, 512])
)

# Ensure batch_size is a clean divisor of buffer size
total_buffer_size = args.n_steps * env.num_envs
effective_batch_size = args.batch_size
while total_buffer_size % effective_batch_size != 0 and effective_batch_size > 64:
    effective_batch_size //= 2

print(f"Total Rollout Buffer: {total_buffer_size} | Minibatch Size: {effective_batch_size}")

model = None
try:
    if args.resume_model_path is None:
        model = PPO(
            "MultiInputPolicy",
            env,
            learning_rate=learning_rate,
            n_steps=args.n_steps,
            batch_size=effective_batch_size,
            n_epochs=args.n_epochs,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            clip_range=args.clip_range,
            ent_coef=args.ent_coef,
            vf_coef=0.5,
            max_grad_norm=0.5,
            policy_kwargs=policy_kwargs,
            normalize_advantage=True,
            tensorboard_log=str(run_dir / "tb_logs"),
            verbose=2,
        )
    else:
        path_zip = pathlib.Path(args.resume_model_path).resolve()
        print(f"Loading model: {path_zip}")
        model = PPO.load(str(path_zip), env=env, tensorboard_log=str(run_dir / "tb_logs"))

    if args.inference:
        obs = env.reset()
        for _ in range(args.timesteps):
            action, _state = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
    else:
        callbacks = []

        if args.save_checkpoint_frequency:
            checkpoint_freq = max(1, args.save_checkpoint_frequency // env.num_envs)
            callbacks.append(
                CheckpointCallback(
                    save_freq=checkpoint_freq,
                    save_path=str(path_checkpoint),
                    name_prefix=args.experiment_name,
                )
            )

        check_freq = max(1, args.best_model_check_freq // env.num_envs)
        callbacks.append(
            RollingBestModelCallback(
                check_freq_steps=check_freq,
                save_path=str(path_best_model),
                start_step=args.best_model_start_step,
                min_episodes=5,
                verbose=1,
            )
        )

        model.learn(
            total_timesteps=args.timesteps,
            tb_log_name="run",
            callback=CallbackList(callbacks),
        )

except (KeyboardInterrupt, ConnectionError, ConnectionResetError) as e:
    print(f"\nTraining interrupted ({type(e).__name__}). Executing cleanup/save...")
finally:
    cleanup(model, env, run_dir)