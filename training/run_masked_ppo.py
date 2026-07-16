"""
Runner script: trains MaskedPPO against a Godot RL Agents environment.
Usage:
    python run_masked_ppo.py --env_path /path/to/your/godot_binary --num_envs 4
Requirements:
    pip install godot-rl torch numpy
"""
import argparse
import os
import time
from collections import deque
import numpy as np
import torch
from godot_rl.core.godot_env import GodotEnv
from ppo_masked import MaskedPPO, PPOConfig


def split_obs_and_mask(obs_dict, num_actions):
    """
    Converts godot_rl's per-step obs (which can be a list, tuple, dict, or flat array)
    into two aligned numpy arrays: obs (N, obs_dim) and mask (N, num_actions).
    """
    # 1. Gymnasium compatibility: extract the actual observation dict if returned as a (obs, info) tuple
    if isinstance(obs_dict, tuple):
        obs_dict = obs_dict[0]
    # 2. Handle vectorized list of environments (e.g. list of agent observation dictionaries)
    if isinstance(obs_dict, list):
        all_obs = []
        all_masks = []
        for item in obs_dict:
            # Recursively split each environment's dictionary
            item_obs, item_mask = split_obs_and_mask(item, num_actions)
            # Standardize shapes to 1D before stacking
            if len(item_obs.shape) > 1:
                item_obs = item_obs.squeeze(0)
            if len(item_mask.shape) > 1:
                item_mask = item_mask.squeeze(0)
            all_obs.append(item_obs)
            all_masks.append(item_mask)
        return np.stack(all_obs, axis=0), np.stack(all_masks, axis=0)
    # 3. Handle dictionary structures
    if isinstance(obs_dict, dict):
        # Case A: Nested under agent names first, e.g. {"Drone_AI_0": {"obs": ..., "action_mask": ...}}
        first_key = list(obs_dict.keys())[0]
        if isinstance(obs_dict[first_key], dict) and "obs" in obs_dict[first_key]:
            all_obs = []
            all_masks = []
            for agent_key in obs_dict:
                all_obs.append(obs_dict[agent_key]["obs"])
                all_masks.append(obs_dict[agent_key]["action_mask"])
            return np.asarray(all_obs, dtype=np.float32), np.asarray(all_masks, dtype=np.float32)
        # Case B: Direct dict containing 'obs' and 'action_mask' keys
        if "obs" in obs_dict:
            obs_val = obs_dict["obs"]
            mask_val = obs_dict["action_mask"]
            # Sub-case: Keys are further nested by agent ID, e.g. {"obs": {"Drone_AI_0": ...}}
            if isinstance(obs_val, dict):
                all_obs = list(obs_val.values())
                all_masks = list(mask_val.values())
                return np.asarray(all_obs, dtype=np.float32), np.asarray(all_masks, dtype=np.float32)
            # Sub-case: Direct arrays
            return np.asarray(obs_val, dtype=np.float32), np.asarray(mask_val, dtype=np.float32)
    # --- Fallback: flat vector with mask packed into the tail ---
    try:
        flat = np.asarray(obs_dict, dtype=np.float32)
        # If flat is 1-dimensional, expand to 2D to support column indexing
        if len(flat.shape) == 1:
            flat = np.expand_dims(flat, axis=0)
        obs = flat[:, :-num_actions]
        mask = flat[:, -num_actions:]
        return obs, mask
    except Exception as e:
        raise TypeError(
            f"Failed to split observation. Received type {type(obs_dict)} "
            f"with keys {list(obs_dict.keys()) if isinstance(obs_dict, dict) else 'N/A'}. "
            f"Error: {e}"
        )


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument(
        "--env_path",
        default=None,
        type=str,
        help="Path to exported Godot binary. Leave unset to connect to editor.",
    )
    parser.add_argument(
        "--num_envs",
        type=int,
        default=1,
        help="How many parallel env instances to launch. Must be 1 when --env_path is not set.",
    )
    parser.add_argument(
        "--num_actions",
        type=int,
        default=None,
        help="Size of discrete action space. Auto-detected from Godot environment if unset."
    )
    parser.add_argument(
        "--obs_dim",
        type=int,
        default=None,
        help="Size of target feature vector. Auto-detected from Godot environment if unset."
    )
    parser.add_argument("--num_steps", type=int, default=256,
                        help="Rollout length per env before each PPO update")
    parser.add_argument("--total_timesteps", type=int, default=2_000_000)
    parser.add_argument("--speedup", type=int, default=1,
                        help="Godot physics speedup factor. Only applies with --env_path set.")
    parser.add_argument(
        "--action_repeat",
        default=None,
        type=int,
        help="Sends action/gets obs every n frames only.",
    )
    parser.add_argument(
        "--viz",
        action="store_true",
        default=False,
        help="Show the simulation window during training.",
    )
    parser.add_argument(
        "--inference",
        action="store_true",
        default=False,
        help="Run inference instead of training.",
    )
    parser.add_argument("--save_path", type=str, default="masked_ppo_model.pt")
    parser.add_argument(
        "--resume_model_path",
        default=None,
        type=str,
        help="Path to checkpoint saved via --save_path to resume training from.",
    )
    parser.add_argument("--log_every", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.env_path is None and args.num_envs != 1:
        raise ValueError(
            "In-editor training (--env_path unset) only supports a single environment. "
            "Set --num_envs 1, or provide --env_path to run parallel exported binaries."
        )

    if args.env_path is None:
        print(
            "No --env_path given: connecting to the Godot editor instead. "
            "Press Play in the Godot editor now if you haven't already."
        )
    else:
        print(f"Launching {args.num_envs} instance(s) of: {args.env_path}")

    # Establish the connection to Godot to acquire environment configuration
    env = GodotEnv(
        env_path=args.env_path,
        n_parallel=args.num_envs,
        speedup=args.speedup,
        action_repeat=args.action_repeat,
        show_window=args.viz,
        seed=args.seed,
    )

    # =========================================================================
    # DYNAMIC CONFIGURATION OF OBSERVATION AND ACTION SPACE DIMENSIONS
    # =========================================================================
    num_actions = args.num_actions
    if num_actions is None:
        action_space = env.action_space
        # If godot-rl returns spaces wrapped in a list/tuple, unpack the first index
        if isinstance(action_space, (list, tuple)) and len(action_space) > 0:
            action_space = action_space[0]
        if isinstance(action_space, dict):
            if "flight_waypoint" in action_space:
                num_actions = int(action_space["flight_waypoint"]["size"])
            else:
                first_key = list(action_space.keys())[0]
                num_actions = int(action_space[first_key]["size"])
        elif hasattr(action_space, "n"):
            num_actions = action_space.n
        elif hasattr(action_space, "spaces") and "flight_waypoint" in action_space.spaces:
            num_actions = action_space.spaces["flight_waypoint"].n
        else:
            num_actions = 78  # Safe fallback
        print(f"Auto-detected action space size (num_actions): {num_actions}")

    obs_dim = args.obs_dim
    if obs_dim is None:
        obs_space = env.observation_space
        # If godot-rl returns spaces wrapped in a list/tuple, unpack the first index
        if isinstance(obs_space, (list, tuple)) and len(obs_space) > 0:
            obs_space = obs_space[0]
        if isinstance(obs_space, dict):
            if "obs" in obs_space:
                obs_dim = int(obs_space["obs"]["size"][0])
            else:
                first_key = list(obs_space.keys())[0]
                obs_dim = int(obs_space[first_key]["size"][0])
        elif hasattr(obs_space, "spaces") and "obs" in obs_space.spaces:
            obs_dim = obs_space.spaces["obs"].shape[0]
        elif hasattr(obs_space, "shape"):
            obs_dim = obs_space.shape[0]
        else:
            obs_dim = 19  # Safe fallback
        print(f"Auto-detected observation space dimensions (obs_dim): {obs_dim}")
    # =========================================================================

    cfg = PPOConfig(
        obs_dim=obs_dim,
        num_actions=num_actions,
        num_envs=args.num_envs,
        num_steps=args.num_steps,
        total_timesteps=args.total_timesteps,
        seed=args.seed,
    )
    agent = MaskedPPO(cfg)

    if args.resume_model_path is not None:
        print(f"Resuming from checkpoint: {os.path.abspath(args.resume_model_path)}")
        agent.load(args.resume_model_path)

    # Handle Inference Mode
    if args.inference:
        print("Running in inference mode deterministically...")
        obs_raw = env.reset()
        obs, mask = split_obs_and_mask(obs_raw, num_actions)
        for i in range(args.total_timesteps):
            action, _, _, _, _ = agent.get_action(obs, mask)
            # Reshape action to (1, num_envs) for godot-rl
            step_action = np.expand_dims(action, axis=0)
            next_obs_raw, reward, next_done, info = env.step(step_action)
            obs, mask = split_obs_and_mask(next_obs_raw, num_actions)
            time.sleep(0.01)
        env.close()
        print("Inference run completed.")
        return

    # Handle Training Mode
    obs_raw = env.reset()
    obs, mask = split_obs_and_mask(obs_raw, num_actions)
    done = np.zeros(args.num_envs, dtype=np.float32)

    # Episodic tracking buffers
    episode_rewards_history = deque(maxlen=100)
    episode_lengths_history = deque(maxlen=100)
    current_rewards = np.zeros(args.num_envs, dtype=np.float32)
    current_lengths = np.zeros(args.num_envs, dtype=np.float32)

    global_step = 0
    update_idx = 0
    start_time = time.time()

    while global_step < cfg.total_timesteps:
        for step in range(cfg.num_steps):
            action, logprob, value, obs_t, mask_t = agent.get_action(obs, mask)
            # Reshape action to (1, num_envs) for godot-rl
            step_action = np.expand_dims(action, axis=0)
            next_obs_raw, reward, next_terminated, next_truncated, info = env.step(step_action)
            next_obs, next_mask = split_obs_and_mask(next_obs_raw, num_actions)

            # Convert parameters safely to numpy arrays to handle vectorized returns
            rewards_arr = np.asarray(reward, dtype=np.float32)
            terminated_arr = np.asarray(next_terminated, dtype=bool)
            truncated_arr = np.asarray(next_truncated, dtype=bool)
            dones_bool = np.logical_or(terminated_arr, truncated_arr)

            # Accumulate current trajectory statistics
            current_rewards += rewards_arr
            current_lengths += 1.0

            # Store stats for any parallel environment that completed this step
            for env_idx, is_done in enumerate(dones_bool):
                if is_done:
                    episode_rewards_history.append(current_rewards[env_idx])
                    episode_lengths_history.append(current_lengths[env_idx])
                    current_rewards[env_idx] = 0.0
                    current_lengths[env_idx] = 0.0

            agent.buffer.add(
                obs_t,
                mask_t,
                torch.as_tensor(action, device=agent.device),
                logprob,
                torch.as_tensor(rewards_arr, dtype=torch.float32, device=agent.device),
                torch.as_tensor(done, dtype=torch.float32, device=agent.device),
                value,
            )

            obs, mask = next_obs, next_mask
            done = dones_bool.astype(np.float32)

        global_step += args.num_envs * cfg.num_steps
        stats = agent.update(obs, done)
        update_idx += 1

        if update_idx % args.log_every == 0:
            elapsed = time.time() - start_time
            sps = int(global_step / elapsed)
            
            # Calculate mean statistics over the history of completed episodes
            mean_reward = np.mean(episode_rewards_history) if len(episode_rewards_history) > 0 else 0.0
            mean_length = np.mean(episode_lengths_history) if len(episode_lengths_history) > 0 else 0.0

            print(
                f"step={global_step:>9} update={update_idx:>5} "
                f"sps={sps:>6} pg_loss={stats['pg_loss']:.4f} "
                f"v_loss={stats['v_loss']:.4f} entropy={stats['entropy']:.4f} "
                f"kl={stats['approx_kl']:.5f} clipfrac={stats['clipfrac']:.3f} "
                f"ep_rew_mean={mean_reward:.2f} ep_len_mean={mean_length:.1f}"
            )

        if update_idx % 20 == 0:
            agent.save(args.save_path)
            print(f"Saved checkpoint to {args.save_path}")

    agent.save(args.save_path)
    env.close()
    print(f"Training complete. Final model saved to {args.save_path}")


if __name__ == "__main__":
    main()