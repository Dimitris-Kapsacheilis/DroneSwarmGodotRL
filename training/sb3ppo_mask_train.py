import argparse
import os
import pathlib
from typing import Callable

import numpy as np
from sb3_contrib import MaskablePPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.vec_env import VecEnvWrapper
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor

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
    help="The Godot binary to use, do not include for in editor training",
)
parser.add_argument(
    "--experiment_dir",
    default="/Users/jimycool/DroneSwarmGodotRL/logs",
    type=str,
    help="The name of the experiment directory, in which the tensorboard logs and checkpoints (if enabled) are stored.",
)
parser.add_argument(
    "--experiment_name",
    default="experiment",
    type=str,
    help="The name of the experiment, which will be displayed in tensorboard and for checkpoint directory and name (if enabled).",
)
parser.add_argument("--seed", type=int, default=0, help="seed of the experiment")
parser.add_argument(
    "--resume_model_path",
    default=None,
    type=str,
    help="The path to a model file previously saved. Use this to resume training or infer from a saved model.",
)
parser.add_argument(
    "--save_model_path",
    default=None,
    type=str,
    help="The path to use for saving the trained model after training is complete.",
)
parser.add_argument(
    "--save_checkpoint_frequency",
    default=None,
    type=int,
    help="If set, will save checkpoints every 'frequency' environment steps.",
)
parser.add_argument(
    "--onnx_export_path",
    default=None,
    type=str,
    help="If included, will export onnx file after training to the path specified.",
)
parser.add_argument(
    "--timesteps",
    default=1_000_000,
    type=int,
    help="The number of environment steps to train for.",
)
parser.add_argument(
    "--inference",
    default=False,
    action="store_true",
    help="Instead of training, it will run inference on a loaded model for --timesteps steps.",
)
parser.add_argument(
    "--linear_lr_schedule",
    default=False,
    action="store_true",
    help="Use a linear LR schedule for training.",
)
parser.add_argument(
    "--viz",
    action="store_true",
    help="If set, the simulation will be displayed in a window during training.",
    default=False,
)
parser.add_argument("--speedup", default=1, type=int, help="Whether to speed up the physics in the env")
parser.add_argument(
    "--action_repeat",
    default=None,
    type=int,
    help="Sends action/gets obs every n frames only.",
)
parser.add_argument(
    "--n_parallel",
    default=1,
    type=int,
    help="How many instances of the environment executable to launch.",
)
parser.add_argument("--learning_rate", default=0.0003, type=float, help="The learning rate (default 0.0003)")
parser.add_argument(
    "--n_steps",
    default=64,
    type=int,
    help="Number of steps to run for each environment per update.",
)
parser.add_argument(
    "--batch_size",
    default=64,
    type=int,
    help="The minibatch size.",
)
parser.add_argument(
    "--ent_coef", default=0.0001, type=float, help="The entropy coefficient (default 0.0001)"
)
parser.add_argument(
    "--clip_range",
    default=0.2,
    type=float,
    help="The clipping range.",
)

args, extras = parser.parse_known_args()

# =====================================================
# CUSTOM WRAPPER FOR EXTRUDING MASKS
# =====================================================
class GodotActionMaskWrapper(VecEnvWrapper):
    """
    VecEnvWrapper designed to intercept the dictionary observations from Godot, 
    separating the target 'obs' from the 'action_mask' array and exposing 
    the active action masks to the MaskablePPO interface.
    """
    def __init__(self, venv):
        super().__init__(venv)
        self.observation_space = venv.observation_space.spaces["obs"]
        self.current_masks = np.ones((venv.num_envs, 78), dtype=np.float32)

    def reset(self):
        obs = self.venv.reset()
        self.current_masks = np.array(obs["action_mask"], dtype=np.float32)
        return obs["obs"]

    def step_wait(self):
        obs, rewards, dones, infos = self.venv.step_wait()
        self.current_masks = np.array(obs["action_mask"], dtype=np.float32)
        return obs["obs"], rewards, dones, infos

    def action_masks(self) -> np.ndarray:
        return self.current_masks

    def env_method(self, method_name: str, *method_args, indices=None, **method_kwargs):
        if method_name == "action_masks":
            if indices is None:
                indices = list(range(self.num_envs))
            elif isinstance(indices, int):
                indices = [indices]
            return [self.current_masks[i] for i in indices]
            
        return self.venv.env_method(method_name, *method_args, indices=indices, **method_kwargs)


def handle_onnx_export():
    if args.onnx_export_path is not None:
        path_onnx = pathlib.Path(args.onnx_export_path).with_suffix(".onnx")
        print("Exporting onnx to: " + os.path.abspath(path_onnx))
        export_model_as_onnx(model, str(path_onnx))


def handle_model_save():
    if args.save_model_path is not None:
        zip_save_path = pathlib.Path(args.save_model_path).with_suffix(".zip")
        print("Saving model to: " + os.path.abspath(zip_save_path))
        model.save(zip_save_path)


def close_env():
    try:
        print("closing env")
        env.close()
    except Exception as e:
        print("Exception while closing env: ", e)


def cleanup():
    handle_onnx_export()
    handle_model_save()
    close_env()


path_checkpoint = os.path.join(args.experiment_dir, args.experiment_name + "_checkpoints")
abs_path_checkpoint = os.path.abspath(path_checkpoint)

if args.save_checkpoint_frequency is not None and os.path.isdir(path_checkpoint):
    raise RuntimeError(
        abs_path_checkpoint + " folder already exists. Change directories or remove the older setup."
    )

if args.inference and args.resume_model_path is None:
    raise parser.error("Using --inference requires --resume_model_path to be set.")

if args.env_path is None and args.viz:
    print("Info: Using --viz without --env_path set has no effect, in-editor training will always render.")

# Initialize the Base Godot Environment
raw_env = StableBaselinesGodotEnv(
    env_path=args.env_path,
    show_window=args.viz,
    seed=args.seed,
    n_parallel=args.n_parallel,
    speedup=args.speedup,
    action_repeat=args.action_repeat,
)

# Apply wrappers: monitor values first, then expose action_masks on the outermost layer
env = VecMonitor(raw_env)
env = GodotActionMaskWrapper(env)


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    def func(progress_remaining: float) -> float:
        return progress_remaining * initial_value
    return func


if args.resume_model_path is None:
    learning_rate = args.learning_rate if not args.linear_lr_schedule else linear_schedule(args.learning_rate)

    model: MaskablePPO = MaskablePPO(
        "MlpPolicy",
        env,
        ent_coef=args.ent_coef,
        verbose=2,
        n_steps=args.n_steps,
        tensorboard_log=args.experiment_dir,
        learning_rate=learning_rate,
        batch_size=args.batch_size,
        clip_range=args.clip_range,
    )
else:
    path_zip = pathlib.Path(args.resume_model_path)
    print("Loading model: " + os.path.abspath(path_zip))
    model = MaskablePPO.load(path_zip, env=env, tensorboard_log=args.experiment_dir)

if args.inference:
    obs = env.reset()
    for i in range(args.timesteps):
        action_masks = env.action_masks()
        action, _state = model.predict(obs, action_masks=action_masks, deterministic=True)
        obs, reward, done, info = env.step(action)
        print(done)
else:
    learn_arguments = dict(total_timesteps=args.timesteps, tb_log_name=args.experiment_name)
    if args.save_checkpoint_frequency:
        print("Checkpoint saving enabled. Checkpoints will be saved to: " + abs_path_checkpoint)
        checkpoint_callback = CheckpointCallback(
            save_freq=(args.save_checkpoint_frequency // env.num_envs),
            save_path=path_checkpoint,
            name_prefix=args.experiment_name,
        )
        learn_arguments["callback"] = checkpoint_callback
    try:
        model.learn(**learn_arguments)
    except (KeyboardInterrupt, ConnectionError, ConnectionResetError):
        print("""Training interrupted. Saving states if directories are configured.""")
    finally:
        cleanup()