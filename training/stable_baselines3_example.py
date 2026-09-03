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
    help="Display simulation window during training (caps FPS to display refresh rate)",
    default=False,
)
parser.add_argument(
    "--speedup", 
    default=8, 
    type=int, 
    help="Physics engine speedup factor (e.g. 4, 8, 16)"
)
parser.add_argument(
    "--action_repeat",
    default=None,
    type=int,
    help="Frame skip count (action repeat)",
)
parser.add_argument(
    "--n_parallel",
    default=8,  # Default to 8 instances (8 * 3 = 24 parallel agents)
    type=int,
    help="Number of parallel Godot executable instances (4 to 32)",
)
parser.add_argument("--learning_rate", default=3e-4, type=float, help="Initial learning rate")

# --- TUNED FOR 3 DRONES, 4-32 PARALLEL ENVS, 1.2K-5K STEP EPISODES ---
parser.add_argument(
    "--n_steps",
    default=1024,  # Rollout length per drone. (Total buffer = n_steps * n_parallel * 3)
    type=int,
    help="Number of steps per drone per rollout",
)
parser.add_argument(
    "--batch_size",
    default=2048,  # Increased from 512 for large multi-agent rollout stability
    type=int,
    help="Minibatch size for PPO updates",
)
parser.add_argument(
    "--n_epochs",
    default=5,  # Reduced from 10 to prevent over-optimizing massive rollout buffers
    type=int,
    help="Number of optimization epochs per rollout",
)
parser.add_argument(
    "--ent_coef",
    default=0.003,  # Slight reduction for steady late-stage precision
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
    default=0.98,  # High lambda keeps variance low across long episodes
    type=float,
    help="GAE lambda",
)
parser.add_argument(
    "--gamma",
    default=0.998,  # High gamma critical for 1.2k-5k step long-horizon credit assignment
    type=float,
    help="Discount factor",
)

args, extras = parser.parse_known_args()


def handle_onnx_export(model):
    if args.onnx_export_path is not None:
        path_onnx = pathlib.Path(args.onnx_export_path).with_suffix(".onnx")
        print(f"Exporting ONNX to: {os.path.abspath(path_onnx)}")
        export_model_as_onnx(model, str(path_onnx))


def handle_model_save(model, env):
    if args.save_model_path is not None:
        zip_save_path = pathlib.Path(args.save_model_path).with_suffix(".zip")
        print(f"Saving model to: {os.path.abspath(zip_save_path)}")
        model.save(zip_save_path)
        
        norm_path = zip_save_path.parent / f"{zip_save_path.stem}_vec_normalize.pkl"
        print(f"Saving VecNormalize stats to: {os.path.abspath(norm_path)}")
        env.save(str(norm_path))


def close_env(env):
    try:
        print("Closing environment...")
        env.close()
    except Exception as e:
        print(f"Exception while closing env: {e}")


def cleanup(model, env):
    if model is not None:
        handle_onnx_export(model)
        handle_model_save(model, env)
    if env is not None:
        close_env(env)


path_checkpoint = os.path.join(args.experiment_dir, args.experiment_name + "_checkpoints")
abs_path_checkpoint = os.path.abspath(path_checkpoint)

if args.save_checkpoint_frequency is not None and os.path.isdir(path_checkpoint):
    print(f"Warning: Checkpoint directory {abs_path_checkpoint} exists and will be used.")

if args.inference and args.resume_model_path is None:
    raise parser.error("Using --inference requires --resume_model_path to be set.")

# 1. Initialize Base Godot Environment
env = StableBaselinesGodotEnv(
    env_path=args.env_path,
    show_window=args.viz,
    seed=args.seed,
    n_parallel=args.n_parallel,
    speedup=args.speedup,
    action_repeat=args.action_repeat,
)

# 2. Add Monitor & Normalization Wrappers
env = VecMonitor(env)

norm_path = None
if args.resume_model_path:
    resume_p = pathlib.Path(args.resume_model_path)
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
        norm_reward=True,
        clip_obs=10.0,
        clip_reward=10.0,
        gamma=args.gamma,
    )


def linear_schedule_with_floor(initial_value: float, min_fraction: float = 0.1) -> Callable[[float], float]:
    """Decays to a minimum fraction (e.g. 10%) instead of 0.0 to prevent freeze."""
    def func(progress_remaining: float) -> float:
        return initial_value * (min_fraction + (1.0 - min_fraction) * progress_remaining)
    return func


use_linear_lr = not args.no_linear_lr_schedule
learning_rate = linear_schedule_with_floor(args.learning_rate, min_fraction=0.1) if use_linear_lr else args.learning_rate

# Multi-Agent Coordination Network Architecture (Expanded for Swarm Policy)
policy_kwargs = dict(
    net_arch=dict(pi=[512, 512], vf=[512, 512])
)

# Guard against batch_size > total_buffer_size
total_buffer_size = args.n_steps * env.num_envs
effective_batch_size = min(args.batch_size, total_buffer_size)

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