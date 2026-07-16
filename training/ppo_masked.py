"""
Custom PPO with discrete action masking, written for use with Godot RL Agents.

Assumes the environment observation is a dict with:
    obs["obs"]          -> flat float32 vector (your frontier feature vector)
    obs["action_mask"]  -> float32 vector of shape (num_actions,), 1 = valid, 0 = invalid

If your GodotEnv doesn't emit action_mask yet, see the note at the bottom of
run_masked_ppo.py for the GDScript/Python side changes needed to add it.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from dataclasses import dataclass, field
from typing import Optional


# --------------------------------------------------------------------------
# Network
# --------------------------------------------------------------------------

class MaskedActorCritic(nn.Module):
    def __init__(self, obs_dim: int, num_actions: int, hidden_size: int = 256):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, hidden_size),
            nn.Tanh(),
        )
        self.policy_head = nn.Linear(hidden_size, num_actions)
        self.value_head = nn.Linear(hidden_size, 1)

        # small init on policy head helps early training stability
        nn.init.orthogonal_(self.policy_head.weight, gain=0.01)
        nn.init.constant_(self.policy_head.bias, 0.0)

    def forward(self, obs: torch.Tensor):
        x = self.shared(obs)
        logits = self.policy_head(x)
        value = self.value_head(x).squeeze(-1)
        return logits, value

    def masked_dist(self, obs: torch.Tensor, action_mask: torch.Tensor):
        logits, value = self.forward(obs)
        # Set invalid action logits to -inf (large negative) so softmax ~= 0
        neg_inf = torch.finfo(logits.dtype).min
        masked_logits = torch.where(action_mask.bool(), logits, torch.full_like(logits, neg_inf))
        dist = torch.distributions.Categorical(logits=masked_logits)
        return dist, value


# --------------------------------------------------------------------------
# Rollout buffer
# --------------------------------------------------------------------------

class RolloutBuffer:
    def __init__(self, num_steps, num_envs, obs_dim, num_actions, device):
        self.num_steps = num_steps
        self.num_envs = num_envs
        self.device = device

        self.obs = torch.zeros((num_steps, num_envs, obs_dim), device=device)
        self.masks = torch.zeros((num_steps, num_envs, num_actions), device=device)
        self.actions = torch.zeros((num_steps, num_envs), dtype=torch.long, device=device)
        self.logprobs = torch.zeros((num_steps, num_envs), device=device)
        self.rewards = torch.zeros((num_steps, num_envs), device=device)
        self.dones = torch.zeros((num_steps, num_envs), device=device)
        self.values = torch.zeros((num_steps, num_envs), device=device)

        self.ptr = 0

    def add(self, obs, mask, action, logprob, reward, done, value):
        i = self.ptr
        self.obs[i] = obs
        self.masks[i] = mask
        self.actions[i] = action
        self.logprobs[i] = logprob
        self.rewards[i] = reward
        self.dones[i] = done
        self.values[i] = value
        self.ptr += 1

    def reset(self):
        self.ptr = 0

    def compute_gae(self, last_value, last_done, gamma=0.99, gae_lambda=0.95):
        advantages = torch.zeros_like(self.rewards)
        last_gae = 0.0
        for t in reversed(range(self.num_steps)):
            if t == self.num_steps - 1:
                next_nonterminal = 1.0 - last_done
                next_value = last_value
            else:
                next_nonterminal = 1.0 - self.dones[t + 1]
                next_value = self.values[t + 1]
            delta = self.rewards[t] + gamma * next_value * next_nonterminal - self.values[t]
            last_gae = delta + gamma * gae_lambda * next_nonterminal * last_gae
            advantages[t] = last_gae
        returns = advantages + self.values
        return advantages, returns


# --------------------------------------------------------------------------
# PPO trainer
# --------------------------------------------------------------------------

@dataclass
class PPOConfig:
    obs_dim: int
    num_actions: int
    num_envs: int = 4
    num_steps: int = 256          # steps per env per rollout
    total_timesteps: int = 2_000_000
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_coef: float = 0.2
    ent_coef: float = 0.01
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5
    update_epochs: int = 4
    num_minibatches: int = 4
    target_kl: Optional[float] = 0.02
    hidden_size: int = 256
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    seed: int = 0


class MaskedPPO:
    def __init__(self, config: PPOConfig):
        self.cfg = config
        torch.manual_seed(config.seed)
        np.random.seed(config.seed)

        self.device = torch.device(config.device)
        self.model = MaskedActorCritic(
            config.obs_dim, config.num_actions, config.hidden_size
        ).to(self.device)
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=config.learning_rate)

        self.buffer = RolloutBuffer(
            config.num_steps, config.num_envs, config.obs_dim, config.num_actions, self.device
        )

    @torch.no_grad()
    def get_action(self, obs: np.ndarray, action_mask: np.ndarray):
        """obs: (num_envs, obs_dim), action_mask: (num_envs, num_actions)"""
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        mask_t = torch.as_tensor(action_mask, dtype=torch.float32, device=self.device)

        dist, value = self.model.masked_dist(obs_t, mask_t)
        action = dist.sample()
        logprob = dist.log_prob(action)
        return (
            action.cpu().numpy(),
            logprob,
            value,
            obs_t,
            mask_t,
        )

    def update(self, last_obs: np.ndarray, last_done: np.ndarray):
        cfg = self.cfg
        with torch.no_grad():
            last_obs_t = torch.as_tensor(last_obs, dtype=torch.float32, device=self.device)
            _, last_value = self.model.forward(last_obs_t)
            last_done_t = torch.as_tensor(last_done, dtype=torch.float32, device=self.device)

        advantages, returns = self.buffer.compute_gae(
            last_value, last_done_t, cfg.gamma, cfg.gae_lambda
        )

        b_obs = self.buffer.obs.reshape(-1, cfg.obs_dim)
        b_masks = self.buffer.masks.reshape(-1, cfg.num_actions)
        b_actions = self.buffer.actions.reshape(-1)
        b_logprobs = self.buffer.logprobs.reshape(-1)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = self.buffer.values.reshape(-1)

        # normalize advantages
        b_advantages = (b_advantages - b_advantages.mean()) / (b_advantages.std() + 1e-8)

        batch_size = cfg.num_steps * cfg.num_envs
        minibatch_size = batch_size // cfg.num_minibatches
        inds = np.arange(batch_size)

        clipfracs = []
        for epoch in range(cfg.update_epochs):
            np.random.shuffle(inds)
            approx_kl_epoch = 0.0
            for start in range(0, batch_size, minibatch_size):
                end = start + minibatch_size
                mb_inds = inds[start:end]

                dist, values = self.model.masked_dist(b_obs[mb_inds], b_masks[mb_inds])
                new_logprob = dist.log_prob(b_actions[mb_inds])
                entropy = dist.entropy()

                logratio = new_logprob - b_logprobs[mb_inds]
                ratio = logratio.exp()

                with torch.no_grad():
                    approx_kl = ((ratio - 1) - logratio).mean().item()
                    clipfracs.append(((ratio - 1.0).abs() > cfg.clip_coef).float().mean().item())
                approx_kl_epoch = approx_kl

                mb_adv = b_advantages[mb_inds]

                pg_loss1 = -mb_adv * ratio
                pg_loss2 = -mb_adv * torch.clamp(ratio, 1 - cfg.clip_coef, 1 + cfg.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                v_loss = 0.5 * ((values - b_returns[mb_inds]) ** 2).mean()
                entropy_loss = entropy.mean()

                loss = pg_loss - cfg.ent_coef * entropy_loss + cfg.vf_coef * v_loss

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), cfg.max_grad_norm)
                self.optimizer.step()

            if cfg.target_kl is not None and approx_kl_epoch > cfg.target_kl:
                break

        self.buffer.reset()
        return {
            "pg_loss": pg_loss.item(),
            "v_loss": v_loss.item(),
            "entropy": entropy_loss.item(),
            "approx_kl": approx_kl_epoch,
            "clipfrac": np.mean(clipfracs),
        }

    def save(self, path: str):
        torch.save(self.model.state_dict(), path)

    def load(self, path: str):
        self.model.load_state_dict(torch.load(path, map_location=self.device))