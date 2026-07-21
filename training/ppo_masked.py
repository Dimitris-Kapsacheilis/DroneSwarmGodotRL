"""
Custom PPO with discrete action masking, written for use with Godot RL Agents.

Assumes the environment observation is a dict with:
    obs["obs"]          -> flat float32 vector (your frontier feature vector)
    obs["action_mask"]  -> float32 vector of shape (num_actions,), 1 = valid, 0 = invalid

If your GodotEnv doesn't emit action_mask yet, see the note at the bottom of
run_masked_ppo.py for the GDScript/Python side changes needed to add it.

Changes vs original version:
    - Fixed approx_kl early-stopping to use the epoch-mean KL, not just the last minibatch.
    - Added a running observation normalizer (helps a lot once frontier feature scale drifts
      as coverage progresses).
    - Added PPO2-style value function clipping.
    - Added linear LR / entropy-coef annealing over total_timesteps.
    - Adam eps=1e-5 (CleanRL-style stabilizer) instead of torch default 1e-8.
    - save/load now includes optimizer + obs normalizer state, so training can resume.
    - MaskedActorCritic now takes an injectable encoder, so you can swap in a GNN encoder
      later for MAPPO/Phase 2+ without touching the PPO update loop.
    - Added an assertion (dev-mode) that every action_mask row has >=1 valid action.
"""

import torch
import torch.nn as nn
import numpy as np
from dataclasses import dataclass
from typing import Optional


# --------------------------------------------------------------------------
# Observation normalization
# --------------------------------------------------------------------------

class RunningNorm(nn.Module):
    """Running mean/std normalizer (Welford-style), kept as buffers so it moves
    with the model via .to(device) and is included in state_dict()."""

    def __init__(self, shape, eps: float = 1e-8, clip: float = 10.0):
        super().__init__()
        self.eps = eps
        self.clip = clip
        self.register_buffer("mean", torch.zeros(shape))
        self.register_buffer("var", torch.ones(shape))
        self.register_buffer("count", torch.tensor(eps))

    @torch.no_grad()
    def update(self, x: torch.Tensor):
        batch_mean = x.mean(dim=0)
        batch_var = x.var(dim=0, unbiased=False)
        batch_count = x.shape[0]

        delta = batch_mean - self.mean
        tot_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / tot_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        m2 = m_a + m_b + delta.pow(2) * self.count * batch_count / tot_count
        new_var = m2 / tot_count

        self.mean.copy_(new_mean)
        self.var.copy_(new_var)
        self.count.copy_(tot_count)

    def forward(self, x: torch.Tensor, update: bool = True) -> torch.Tensor:
        if update and self.training:
            self.update(x)
        normed = (x - self.mean) / torch.sqrt(self.var + self.eps)
        return torch.clamp(normed, -self.clip, self.clip)


# --------------------------------------------------------------------------
# Network
# --------------------------------------------------------------------------

class MLPEncoder(nn.Module):
    """Default encoder. Swap this out (e.g. for a GNN encoder) when moving to
    MAPPO/Phase 2+ -- MaskedActorCritic doesn't care what's inside, only the
    output dim."""

    def __init__(self, obs_dim: int, hidden_size: int = 256):
        super().__init__()
        self.out_dim = hidden_size
        self.net = nn.Sequential(
            nn.Linear(obs_dim, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, hidden_size),
            nn.Tanh(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class MaskedActorCritic(nn.Module):
    def __init__(self, encoder: nn.Module, encoder_out_dim: int, num_actions: int):
        super().__init__()
        self.encoder = encoder
        self.policy_head = nn.Linear(encoder_out_dim, num_actions)
        self.value_head = nn.Linear(encoder_out_dim, 1)

        # small init on policy head helps early training stability
        nn.init.orthogonal_(self.policy_head.weight, gain=0.01)
        nn.init.constant_(self.policy_head.bias, 0.0)

    def forward(self, obs: torch.Tensor):
        x = self.encoder(obs)
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

    # New: annealing + normalization toggles
    anneal_lr: bool = True
    anneal_ent_coef: bool = True
    normalize_obs: bool = True
    clip_vloss: bool = True
    debug_assertions: bool = True   # set False once you trust your env's action_mask


class MaskedPPO:
    def __init__(self, config: PPOConfig, encoder: Optional[nn.Module] = None):
        self.cfg = config
        torch.manual_seed(config.seed)
        np.random.seed(config.seed)

        self.device = torch.device(config.device)

        # Allows swapping in e.g. a GNN encoder later for MAPPO without touching
        # anything below this line.
        if encoder is None:
            encoder = MLPEncoder(config.obs_dim, config.hidden_size)
        encoder_out_dim = getattr(encoder, "out_dim", config.hidden_size)

        self.model = MaskedActorCritic(
            encoder, encoder_out_dim, config.num_actions
        ).to(self.device)

        # eps=1e-5 (vs torch default 1e-8) is a well-documented PPO stabilizer (CleanRL etc.)
        self.optimizer = torch.optim.Adam(
            self.model.parameters(), lr=config.learning_rate, eps=1e-5
        )

        self.obs_normalizer: Optional[RunningNorm] = None
        if config.normalize_obs:
            self.obs_normalizer = RunningNorm((config.obs_dim,)).to(self.device)

        self.buffer = RolloutBuffer(
            config.num_steps, config.num_envs, config.obs_dim, config.num_actions, self.device
        )

        # tracks progress for LR/entropy annealing; call `notify_timesteps` from your
        # training loop each rollout, or pass `timesteps_so_far` into update() directly.
        self._timesteps_so_far = 0

    def _normalize(self, obs_t: torch.Tensor, update: bool) -> torch.Tensor:
        if self.obs_normalizer is None:
            return obs_t
        return self.obs_normalizer(obs_t, update=update)

    def _current_frac_remaining(self) -> float:
        frac = 1.0 - (self._timesteps_so_far / max(1, self.cfg.total_timesteps))
        return max(0.0, min(1.0, frac))

    @torch.no_grad()
    def get_action(self, obs: np.ndarray, action_mask: np.ndarray):
        """obs: (num_envs, obs_dim), action_mask: (num_envs, num_actions)"""
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        mask_t = torch.as_tensor(action_mask, dtype=torch.float32, device=self.device)

        if self.cfg.debug_assertions:
            assert (mask_t.sum(dim=-1) > 0).all(), (
                "action_mask row with no valid actions -- check your frontier "
                "top-K candidate generation for an edge case producing zero candidates."
            )

        obs_t = self._normalize(obs_t, update=True)

        dist, value = self.model.masked_dist(obs_t, mask_t)
        action = dist.sample()
        logprob = dist.log_prob(action)
        return (
            action.cpu().numpy(),
            logprob,
            value,
            obs_t,   # NOTE: this is the *normalized* obs -- store this in the buffer,
                     # not the raw obs, so training sees consistent scale.
            mask_t,
        )

    def update(self, last_obs: np.ndarray, last_done: np.ndarray, timesteps_so_far: Optional[int] = None):
        cfg = self.cfg
        if timesteps_so_far is not None:
            self._timesteps_so_far = timesteps_so_far

        # --- LR / entropy annealing ---
        frac_remaining = self._current_frac_remaining()
        if cfg.anneal_lr:
            lr_now = cfg.learning_rate * frac_remaining
            for pg in self.optimizer.param_groups:
                pg["lr"] = lr_now
        ent_coef_now = cfg.ent_coef * frac_remaining if cfg.anneal_ent_coef else cfg.ent_coef

        with torch.no_grad():
            last_obs_t = torch.as_tensor(last_obs, dtype=torch.float32, device=self.device)
            last_obs_t = self._normalize(last_obs_t, update=False)
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
        approx_kls = []
        pg_loss = v_loss = entropy_loss = torch.tensor(0.0)

        for epoch in range(cfg.update_epochs):
            np.random.shuffle(inds)
            epoch_kls = []
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
                epoch_kls.append(approx_kl)

                mb_adv = b_advantages[mb_inds]

                pg_loss1 = -mb_adv * ratio
                pg_loss2 = -mb_adv * torch.clamp(ratio, 1 - cfg.clip_coef, 1 + cfg.clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                if cfg.clip_vloss:
                    v_unclipped = (values - b_returns[mb_inds]) ** 2
                    v_clipped_pred = b_values[mb_inds] + torch.clamp(
                        values - b_values[mb_inds], -cfg.clip_coef, cfg.clip_coef
                    )
                    v_clipped = (v_clipped_pred - b_returns[mb_inds]) ** 2
                    v_loss = 0.5 * torch.max(v_unclipped, v_clipped).mean()
                else:
                    v_loss = 0.5 * ((values - b_returns[mb_inds]) ** 2).mean()

                entropy_loss = entropy.mean()

                loss = pg_loss - ent_coef_now * entropy_loss + cfg.vf_coef * v_loss

                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(), cfg.max_grad_norm)
                self.optimizer.step()

            approx_kls.append(np.mean(epoch_kls))
            # Fixed: compare the epoch's mean KL (not just the last minibatch's) against target_kl.
            if cfg.target_kl is not None and approx_kls[-1] > cfg.target_kl:
                break

        self.buffer.reset()
        return {
            "pg_loss": pg_loss.item(),
            "v_loss": v_loss.item(),
            "entropy": entropy_loss.item(),
            "approx_kl": approx_kls[-1],
            "clipfrac": np.mean(clipfracs),
            "lr": self.optimizer.param_groups[0]["lr"],
            "ent_coef": ent_coef_now,
        }

    def save(self, path: str):
        state = {
            "model": self.model.state_dict(),
            "optimizer": self.optimizer.state_dict(),
            "timesteps_so_far": self._timesteps_so_far,
        }
        if self.obs_normalizer is not None:
            state["obs_normalizer"] = self.obs_normalizer.state_dict()
        torch.save(state, path)

    def load(self, path: str):
        state = torch.load(path, map_location=self.device)
        self.model.load_state_dict(state["model"])
        if "optimizer" in state:
            self.optimizer.load_state_dict(state["optimizer"])
        if "obs_normalizer" in state and self.obs_normalizer is not None:
            self.obs_normalizer.load_state_dict(state["obs_normalizer"])
        self._timesteps_so_far = state.get("timesteps_so_far", 0)