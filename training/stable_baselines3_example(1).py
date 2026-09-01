import argparse
import os
import pathlib
from typing import Callable

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
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
    help="Directory to store tensorboard logs and checkpoints",
)
parser.add_argument(
    "--experiment_name",
    default="experiment",
    type=str,
    help="Name of the experiment",
)
parser.add_argument("--seed", type=int, default=0, help="Seed of the experiment")
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
    default=None,
    type=int,
    help="Save checkpoints every N environment steps",
)
parser.add_argument(
    "--onnx_export_path",
    default=None,
    type=str,
    help="Path to export the ONNX model after training",
)
parser.add_argument(
    "--timesteps",
    default=1_000_000,
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
    "--linear_lr_schedule",
    default=False,
    action="store_true",
    help="Use a linear LR decay schedule",
)
parser.add_argument(
    "--viz",
    action="store_true",
    help="Display simulation window during training",
    default=False,
)
parser.add_argument("--speedup", default=1, type=int, help="Physics speedup factor")
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
parser.add_argument("--learning_rate", default=3e-4, type=float, help="Learning rate")
# --- FIXED HYPERPARAMETERS BELOW ---
parser.add_argument(
    "--n_steps",
    default=2048,  # Increased from 64 to 2048 for stable GAE estimates
    type=int,
    help="Number of steps to run for each environment per rollout",
)
parser.add_argument(
    "--batch_size",
    default=128,  # Increased from 64 to 128
    type=int,
    help="Minibatch size for PPO updates",
)
parser.add_argument(
    "--n_epochs",
    default=10,
    type=int,
    help="Number of optimization epochs per rollout",
)
parser.add_argument(
    "--ent_coef",
    default=0.005,  # Increased from 0.0001 to promote exploration
    type=float,
    help="Entropy coefficient",
)
parser.add_argument(
    "--clip_range",
    default=0.2,
    type=float,
    help="PPO surrogate clipping range",
)
parser.add_argument(
    "--gae_lambda",
    default=0.95,
    type=float,
    help="Factor for trade-off of bias vs variance for GAE",
)
parser.add_argument(
    "--gamma",
    default=0.99,
    type=float,
    help="Discount factor",
)

args, extras = parser.parse_known_args()


def handle_onnx_export(model):
    if args.onnx_export_path is not None:
        path_onnx = pathlib.Path(args.onnx_export_path).with_suffix(".onnx")
        print(f"Exporting ONNX to: {os.path.abspath(path_onnx)}")
        export_model_as_onnx(model, str(path_onnx))


def handle_model_save(model):
    if args.save_model_path is not None:
        zip_save_path = pathlib.Path(args.save_model_path).with_suffix(".zip")
        print(f"Saving model to: {os.path.abspath(zip_save_path)}")
        model.save(zip_save_path)


def close_env(env):
    try:
        print("Closing environment...")
        env.close()
    except Exception as e:
        print(f"Exception while closing env: {e}")


def cleanup(model, env):
    if model is not None:
        handle_onnx_export(model)
        handle_model_save(model)
    if env is not None:
        close_env(env)


path_checkpoint = os.path.join(args.experiment_dir, args.experiment_name + "_checkpoints")
abs_path_checkpoint = os.path.abspath(path_checkpoint)

if args.save_checkpoint_frequency is not None and os.path.isdir(path_checkpoint):
    raise RuntimeError(
        f"{abs_path_checkpoint} folder already exists. "
        "Use a different --experiment_dir or remove the existing checkpoint directory."
    )

if args.inference and args.resume_model_path is None:
    raise parser.error("Using --inference requires --resume_model_path to be set.")

# 1. Initialize Godot Env
env = StableBaselinesGodotEnv(
    env_path=args.env_path,
    show_window=args.viz,
    seed=args.seed,
    n_parallel=args.n_parallel,
    speedup=args.speedup,
    action_repeat=args.action_repeat,
)

# 2. Add Monitor wrapper
env = VecMonitor(env)


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value
    return func


learning_rate = args.learning_rate if not args.linear_lr_schedule else linear_schedule(args.learning_rate)

model = None
try:
    if args.resume_model_path is None:
        model = PPO(
            "MultiInputPolicy",
            env,
            learning_rate=learning_rate,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            n_epochs=args.n_epochs,
            gamma=args.gamma,
            gae_lambda=args.gae_lambda,
            clip_range=args.clip_range,
            ent_coef=args.ent_coef,
            vf_coef=0.5,
            max_grad_norm=0.5,
            normalize_advantage=True,
            tensorboard_log=args.experiment_dir,
            verbose=2,
        )
    else:
        path_zip = pathlib.Path(args.resume_model_path)
        print(f"Loading model: {os.path.abspath(path_zip)}")
        model = PPO.load(path_zip, env=env, tensorboard_log=args.experiment_dir)

    if args.inference:
        obs = env.reset()
        for _ in range(args.timesteps):
            action, _state = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
    else:
        learn_arguments = dict(total_timesteps=args.timesteps, tb_log_name=args.experiment_name)
        if args.save_checkpoint_frequency:
            print(f"Checkpoint saving enabled. Checkpoints will be saved to: {abs_path_checkpoint}")
            checkpoint_callback = CheckpointCallback(
                save_freq=max(1, args.save_checkpoint_frequency // env.num_envs),
                save_path=path_checkpoint,
                name_prefix=args.experiment_name,
            )
            learn_arguments["callback"] = checkpoint_callback

        model.learn(**learn_arguments)

except (KeyboardInterrupt, ConnectionError, ConnectionResetError) as e:
    print(f"\nTraining interrupted: {e}. Executing cleanup/save...")
finally:
    cleanup(model, env)