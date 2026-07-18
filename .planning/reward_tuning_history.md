# Reward Tuning History
This file tracks the history of issues encountered during RL training and the reward tuning fixes applied.
Always read this file before modifying the reward function to ensure you do not repeat past failed configurations.

## History

**Issue 1**: The agent found a "suicide loophole" where it would sprint forward to earn `progress_reward` and intentionally crash into the wall early (`ep_len_mean` < 200) because the crash penalty was `0.0`.
**Fix 1**: We iteratively increased `termination_penalty` from `0.0` -> `1.0` -> `50.0` -> `500.0` -> `2000.0`.
**Result**: Success! The agent stopped crashing early and survived for up to 450 steps per episode.

**Issue 2**: The agent was surviving but earning massive step penalties (`ep_rew_mean` ~ -2040). It was violently wiggling its steering to avoid the boundaries, triggering huge `jerk_penalty` and `lateral_error` penalties on the straightaways instead of learning smooth cornering.
**Fix 2**: We increased `lateral_error` weight from `0.5` to `2.0` and decreased `jerk_penalty` weight from `0.5` to `0.1`.
**Result**: Failure! The increased `lateral_error` made surviving and wiggling mathematically worse than crashing. The agent reverted to the suicide loophole (`ep_len_mean` dropped to 121, speed spiked to 0.64 m/s).

**Issue 3**: Agent regressed to suicide loophole due to high `lateral_error` penalty.
**Fix 3**: Reverted `lateral_error` from `2.0` back down to `0.5`. Doubled `progress_reward` from `25.0` to `50.0` to massively over-incentivize surviving and moving forward to offset any remaining step penalties.
**Result**: Failure! The agent survived slightly longer (`ep_len_mean` ~195) but still succumbed to the suicide loophole. The massive `termination_penalty` (-2000) combined with accumulated step penalties (-40) made surviving mathematically worse than crashing.

**Issue 4**: The massive `-2000` termination penalty acts as a sparse binary failure that the agent cannot optimize smoothly against. When faced with difficult turns and continuous step penalties (like `lateral_error`), the agent prefers to end the episode early (suicide) rather than live and accumulate more step penalties before an inevitable crash.
**Fix 4**: Applied "Risk-Aware Shaping". Set `termination_penalty` to `0.0`. Enabled the `boundary_penalty` with a weight of `10.0` to provide dense, informative feedback as the agent approaches the walls, steering it away smoothly rather than waiting for a massive terminal penalty.
**Result**: Failed. The agent became reckless, achieving higher speed (~0.62m/s) but `ep_len_mean` dropped to ~176. Without a terminal penalty, the agent ignored the dense boundary shaping because the positive rewards for driving fast outweighed the boundary penalty, and crashing had zero cost.

**Issue 5**: "Risk-Aware Shaping" without a terminal penalty causes reward hacking (the "Endless Runner exploit"). The agent sprints the easy straightaways and intentionally crashes at the difficult curves because dying is just a "free reset" to harvest easy initial rewards again.
**Fix 5**: Re-introduced `termination_penalty` with a moderate weight of `1000.0`. Research confirms that dense boundary shaping MUST be paired with a substantial negative terminal penalty, otherwise the agent learns that collision states are acceptable "neutral" outcomes compared to the effort of navigating correctly.
**Result**: Agent's episode length spiked to ~300 but slowly decayed back to ~166 as it learned to speed up (0.62 m/s). It started ignoring the `boundary_penalty` (weight 10.0) because the `progress_reward` (weight 50.0) outweighed it, causing a new speed-and-crash loophole.

**Issue 6**: Speed reward is 5x higher than safety penalty, incentivizing reckless driving and early crashes at the 500k milestone.
**Fix 6**: Re-balanced rewards based on CMDP/Safety principles. Massively increased `boundary_penalty` from `10.0` to `100.0` to act as a hard safety multiplier/constraint. This makes any proximity to the wall mathematically devastating compared to the 50.0 progress reward.
**Result**: Failed. The agent survived for ~208 steps on average, but accumulated massive negative rewards (`ep_rew_mean` ~ -1700).

**Issue 7**: The `weight` parameters in `arcpro_env_cfg.py` act as multipliers for the native function returns in `rewards.py`. The native `mdp_rew.boundary_penalty` returns `-100.0`, and the `mdp_rew.termination_penalty` returns `-50.0`. By setting their weights to `100.0` and `1000.0` respectively, we inadvertently created astronomical penalties (e.g. `-10,000` per step near a boundary, and `-50,000` for crashing). This mathematically crushed all positive rewards.
**Fix 7**: Normalized the weights. Changed `termination_penalty` weight to `20.0` (yields exactly `-1000.0` upon crash). Changed `boundary` weight to `0.1` (yields exactly `-10.0` per step). This properly balances the Risk-Aware Shaping without overpowering the progress and survival bonuses.
**Result**: Partial Success. At the 250k milestone, the agent reached an `ep_len_mean` of ~200. However, the speed plateaued at ~0.25 m/s.

**Issue 8**: The agent's episode length improved to ~200 at 250k steps but speed remained below 0.3m/s (far below the >0.6 m/s target). The +10.0 survival bonus outweighed the small -1.0 stationary penalty, meaning the agent found an optimal balance driving slowly to maximize survival time while taking minimum risk, a classic 'conservative agent' issue.
**Fix 8**: Updated the stationary penalty threshold from 0.3 m/s to 0.5 m/s, and massively increased its weight to 15.0. This yields a net -5.0 penalty per step if the agent drives under 0.5 m/s, completely cancelling out the survival bonus and forcing it to speed up to gain positive reward.
**Result**: Pending next 250k milestone.

## Architectural Shifts (Context Clear 2026-07-14)

**Shift 1**: Migrated from Stable-Baselines3 to SKRL (`train_skrl.py`). Implemented a custom `TelemetryPPO` class to explicitly write `speed_mps`, `dist_white`, and `dist_yellow` directly to TensorBoard under the 'Telemetry' grouping.

**Shift 2**: The user requested that the HD cameras be kept on (`--enable_cameras`), but SKRL was originally flattening the image into a 150,000-dim array and feeding it into an MLP, which reduced training to 2.8 steps per second.
**Fix**: We implemented a frozen pre-trained `ResNet-18` backbone *inside* the environment wrapper (`SKRLFlattenWrapper`). This evaluates the camera image exactly once per environment step (during rollout) and passes the extracted 512-dimensional feature embedding into the Replay Buffer. The PPO Actor (`ARCProActor`) now acts as a lightning fast MLP over those 512 cached features.

**Shift 3**: Because the architecture shifted from a 12-dimensional telemetry-only input to a 524-dimensional combined input, the old weights became incompatible and the training was **restarted from scratch (Step 0)**.

**Current State**: At the 250k milestone, the agent was driving fast (Speed ~0.66 m/s) but crashing early (Length ~115).
**Issue 9**: The stationary penalty was set to 40.0 in the config instead of the intended 15.0 from Fix 8. This massive penalty (-40.0 net per step if slow) caused the agent to prefer crashing over driving safely at a moderate speed, leading to reckless high-speed driving and early termination to avoid accumulating stationary penalties.
**Fix 9**: Restored the stationary penalty weight to 15.0 in `arcpro_env_cfg.py` to balance it properly against the 10.0 survival bonus (resulting in a -5.0 net penalty per step when slow). Restarted training.

**Issue 10**: Even after Fix 9, the agent was still suffering from "immediate termination" (episode lengths under 200). We realized that while the agent's observation space expanded from 12 to 524 dims (due to ResNet), requiring a brain restart, we accidentally left the strict "Phase 2" penalties (stationary, lateral_error, heading, smoothness, jerk) active. A newborn agent cannot learn forward momentum if it is instantly penalized for imprecise lane tracking and slow speeds.
**Fix 10**: We implemented a Curriculum. We temporarily reverted `arcpro_env_cfg.py` back to a Phase 1 setup (setting lateral_error, stationary, heading, smoothness, and jerk weights to 0.0) while keeping survival and progress rewards high. Restarted training. The agent quickly began surviving for 400-600+ steps at slower, safer speeds (~0.3m/s). We will wait until it masters forward momentum before re-enabling Phase 2 precision penalties.

**Issue 11**: The Phase 1 curriculum initially helped the agent survive for 400-600 steps, but by the 250k milestone, episode lengths decayed to ~169 steps. The agent was suffering from the "Endless Runner exploit" again: because `progress_reward` was very high (weight 50.0), the agent accumulated massive positive reward (+43/step at 0.6 m/s) and found it more profitable to speed down the straightaway and crash (taking a -1000 penalty) than to learn how to corner safely.
**Fix 11**: Massively increased the `termination_penalty` weight from `20.0` to `100.0` (yielding a devastating -5000 penalty upon crash) and halved the `progress_reward` weight from `50.0` to `25.0`. This ensures that crashing will mathematically wipe out the accumulated rewards of a short, fast sprint, forcing the agent to learn to brake and corner to survive long enough to keep its points.
**Result**: Failure. The -5000 penalty created a massive value function cliff, destabilizing training. The agent's `ep_len_mean` crashed to ~93 and speed dropped to 0.17 m/s. The agent learned to barely move to avoid any possibility of triggering the extreme crash penalty.

**Issue 12**: The extreme termination penalty (-5000) prevents the agent from learning effectively. However, reverting it will re-enable the "Endless Runner exploit" where linear progress rewards scale infinitely with speed, making crashing profitable.
**Fix 12**: Applied "Bounded Progress Reward". Reduced `termination_penalty` weight back to `20.0` (-1000) to allow smooth learning. Replaced the linear progress reward with `torch.tanh(mdp_rew.progress_reward(env))` (weight `50.0`). The `tanh` bound limits the maximum reward an agent can earn per step from speeding, removing the incentive to sprint recklessly since going 5x faster only yields ~1.5x the reward. Also reduced `survival_bonus` to `1.0` so the agent must move to gain meaningful points.
**Result**: The agent survived for ~125 steps on average, but speed plateaued at ~0.22 m/s. It became overly conservative.

**Issue 13**: Bounded progress reward combined with a low survival bonus caused the agent to become overly conservative at the 250k milestone (Len: 125, Spd: 0.22 m/s). It did not learn to steer and essentially barely moved before crashing.
**Fix 13**: Re-enabled `stationary` penalty with a moderate weight of `5.0`. This yields a net `-5.0` penalty per step if the agent drives under `0.5 m/s`. Since the survival bonus is only `1.0`, sitting still now results in a net negative reward of `-4.0` per step, forcing the agent to break out of the conservative local minimum and speed up. Restarted training.

**Issue 14 (ROOT CAUSE)**: We added track progress telemetry (`Trk%` and `WPΔ`) to verify whether the agent was actually navigating the track. **It was not.** Even episodes with 1,000+ steps and 1,249 reward showed `WPΔ: 0` — zero waypoints traversed. The agent was sitting near spawn, collecting `survival_bonus` and `tanh(forward_speed)` rewards by wiggling or slowly circling. The `progress_reward` function (based on `root_lin_vel_b[:, 0]` — local body velocity) is fundamentally exploitable. The agent can point any direction, creep forward relative to its own body, and farm points indefinitely without advancing along the track. **Every previous "success" metric (reward, episode length, speed) was meaningless without `WPΔ`.**
**Fix 14**: Replaced `progress_reward` entirely with `waypoint_progress_reward` in `rewards.py`. The new function uses the TrackManager's waypoint index to compute the number of new centerline waypoints traversed per step. This is the ONLY metric that cannot be gamed by circling or wiggling. Set weight to `5.0` (yields ~15 reward/step at 0.5 m/s). Kept `survival_bonus=1.0`, `termination_penalty=20.0`, `stationary=5.0`, `boundary=0.1`. Restarted training.
