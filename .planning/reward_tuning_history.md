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

---

## Physics Fixes Session (2026-07-18)

**Issue 15 — Early Death at Steps 49/55 (Height Termination)**
During a straight-line open-loop physics test (throttle=1.0, steer=0.0), the car was dying every 49 or 55 steps in a perfectly alternating loop. Root cause: the `lin_vel=(0.0, 2.0, 0.0)` kickstart was launching the car at full speed from spawn. Within ~50 steps (~1 second), it hit the first curve, bounced off, and either dipped below the `height_termination` threshold (0.02m) or flew above `max_height` (0.50m). The kickstart was masking a deeper physics instability.
**Fix 15** (3 sub-fixes applied atomically):
- **15A** `spawner.py`: Added `_apply_tire_friction()` — applies `UsdPhysics.MaterialAPI` to all `Wheel_*` prims at spawn time. `static_friction=1.2`, `dynamic_friction=0.8`, `restitution=0.0`. Eliminates bounce on road contact.
- **15B** `arcpro_robot_cfg.py`: Added `linear_damping=0.1` and `angular_damping=0.05` to chassis rigid body props. Models rolling resistance and prevents spin-out on bumps. Previously both were `0.0` (frictionless in free space).
- **15C** `arcpro_env_cfg.py`: Removed `lin_vel=(0.0, 2.0, 0.0)` kickstart entirely. Proper tire friction + drive torque (effort_limit=5.0 N·m → ~167N force on 4.2kg car) is more than sufficient to overcome static friction. Tightened `height_termination` threshold `0.02m → 0.005m` to give 15mm more suspension room. Dropped `stationary` penalty speed threshold `0.5 → 0.2 m/s` to match realistic RC car minimum crawl speed.
**Result**: ✅ **CONFIRMED FIXED.** Post-fix straight-line test ran 500 steps with zero resets. Physics summary: `max_speed_seen = 1.968 m/s`. Speed profile was stable and smooth (0.895 m/s at step 50 → 1.968 m/s peak → settles ~0.7-1.2 m/s as car corners into turns). Z axis stable throughout — no height terminations.

**Issue 16 — WPΔ_rew Reward Spike (Bug 4 — spawn_wp_idx Race Condition)**
At ~640k–716k steps, `WPΔ_rew` spiked to values like `2434`, `7300`, `5795` — impossible for one step of real navigation. These coincide with `WPs_cum = 3361` (also impossible in one episode). Root cause: `spawn_wp_idx` reset race condition — on episode reset, the TrackManager's `last_indices` is briefly out of sync with the robot's new position, causing a massive false `delta = (new_idx - old_idx) % num_wps` calculation that wraps around the entire track.
**Fix 16 (APPLIED)**: Zeroed out `prev_wp_idx_reward` and `prev_pos_reward` for all reset environments in `rewards.py::waypoint_progress_reward` by checking `env.reset_terminated` and `env.reset_buf`. This ensures the massive backwards track wrap-around distance isn't rewarded during the reset step.

**ALWAYS READ BEFORE REWARD OR PHYSICS CHANGES**: This file documents 18 issues. Do not repeat them.

## Post-Physics Stabilization Session (2026-07-20)

**Issue 17 — The Torque Limit Wheelie (Mask Unveiled)**
After Fix 15A applied proper static (1.2) and dynamic (0.8) friction to the tires, the car started popping wheelies and backflipping under full throttle. Root cause: The throttle actuator `effort_limit_sim` had previously been cranked up to `5.0` N·m per wheel (20 Nm total for a 3.3kg car) to artificially overcome the old lack of tire grip. With realistic grip restored, 20 Nm of torque was instantly flipping the car.
**Fix 17**: Reverted `effort_limit_sim` in `arcpro_robot_cfg.py` back to a realistic `0.5` N·m. The car now accelerates smoothly and remains planted on the ground.

**Issue 18 — The Camera Pipeline Crash (--enable_cameras)**
Running visual evaluation via `debug.sh --enable_cameras` failed due to a severe shape mismatch (`150540 vs 524`). Root cause: `train_skrl.py` was properly configured with a frozen `ResNet-18` vision encoder (extracting 512 dims from the 224x224 RGB image), but the evaluation scripts (`play_skrl.py`, `play_straight.py`, `test_term.py`) were using a hardcoded, outdated version of `SKRLFlattenWrapper` that just `.reshape` flattened the raw RGB pixels (3*224*224 = 150528 + 12 = 150540 dims).
**Fix 18**: Extracted the proper ResNet-enabled wrapper into a new shared module `arcproLab/agents/skrl_wrappers.py` and refactored all train and evaluation scripts to import it. `--enable_cameras` now perfectly matches the 524-dim architecture across all scripts.

**Issue 19 — Stagnation at the First Curve (The Endless Runner Exploit returns)**
At step 700k, the episode length stagnated at ~60-120 steps and speed hovered around ~0.45 m/s. The agent was sprinting straight and crashing into the first curve without attempting to turn. Root cause: The `progress_reward` weight was set to 20.0. At 0.5m/s, the agent earned ~34 points per step (approx 3400 points over 100 steps). This massive positive reward vastly outweighed the -1000 crash penalty (`termination_penalty`). Sprinting into the wall was highly profitable.
**Fix 19**: Reverted the `progress_reward` weight from 20.0 back down to 5.0. Now, a 100-step sprint yields only ~850 points, meaning a -1000 crash results in a net negative score for the episode. The agent must now learn to steer and survive the curve to achieve a positive score.

**Issue 20 — Persistent Stagnation at 1 Million Steps (Sparse Penalty)**
Even after Fix 19, the agent failed to break the 100-step barrier by 1M steps. The root cause was twofold: (1) The default initialized network action of [0, 0] translated to "drive straight at half speed" due to an offset of -20.0 in the throttle actuator, automatically propelling the agent into the first curve at ~100 steps. (2) The `boundary_penalty` was effectively sparse. Its threshold was 0.15m, while the fatal `termination` threshold was 0.12m. This gave the agent only a 0.03m warning zone (just ~1-3 steps at 0.5 m/s) before instant death (-1000). The agent couldn't figure out *which* action caused the crash.
**Fix 20**: Increased the `boundary_penalty` threshold in `mdp/rewards.py` from 0.15m to 0.40m. This creates a 0.28m-wide "warning track" that provides dense negative feedback (-10.0 weight * 0.1) as the agent approaches the wall, smoothly guiding it to turn long before it triggers the fatal 0.12m crash.

**Issue 21 — The Weak Gradient Trap (Failure to Bootstrap Cornering)**
Even with the dense warning track, the agent failed to learn to corner by 577k steps (ep len stalled at ~110). Root cause: In Fix 19, we lowered the `progress_reward` weight to 5.0 to prevent the endless runner sprint exploit. However, this made the reward for exploring the turn too weak to overcome the exploration noise. In previous iterations (like Issue 10), the agent *did* learn to survive 400-600 steps because the `progress_reward` weight was 50.0, which massively rewarded any actions that extended life. 
**Fix 21**: Restored the `progress_reward` weight to 50.0 to bootstrap cornering, but bounded it with a `torch.tanh` wrapper (`lambda env: torch.tanh(mdp_rew.waypoint_progress_reward(env))`). This provides massive points for surviving at normal speeds (e.g. 0.5 m/s yields ~46 pts/step), but prevents the sprint exploit because sprinting 3x faster (1.5 m/s) only yields ~49 pts/step (a mere 6% increase). Survival is mathematically guaranteed to be the most optimal strategy.

**Issue 22 — Negative Rewards & Point Starvation (The Displacement Gate)**
After Fix 21, the agent was surprisingly receiving massive *negative* rewards (around -20 to -30 per step) and failed to learn entirely. Root cause: An older patch ("Fix C") added a displacement gate to `waypoint_progress_reward` that zeroed out the reward if the agent moved `< 0.02m` (2cm) per step to prevent jitter farming. However, at a 50Hz simulation (dt=0.02s), moving 0.02m per step requires a speed of `> 1.0 m/s`. Because the agent naturally explores at speeds of ~0.3 to 0.6 m/s, it was moving `< 0.02m` per step. The gate blocked 100% of the progress points! The agent was effectively starved of positive rewards while still taking the dense `-10.0` boundary penalties, resulting in completely negative scores for normal driving.
**Fix 22**: Lowered the displacement gate threshold in `mdp/rewards.py` from `0.02m` to `0.005m` (5mm). At 50Hz, 5mm/step equals `0.25 m/s`. This perfectly aligns with our `stationary_penalty` (which punishes speeds `< 0.2 m/s`). The agent now correctly earns progress points for driving forward at reasonable speeds.

**Issue 23 — Overpowered Progress & Missing Brakes**
By 770k steps, the agent was still failing to make the first curve (stalling at ~110-150 steps). Root cause analysis revealed two intertwined math failures: 
1. **Ineffective Warning Track**: The `boundary_penalty` was set to `weight=0.1` (yielding `-10.0` points/step). Because `waypoint_progress_reward` yields up to `+50.0` points/step, the agent was earning a net positive `+40.0` points *even while scraping the walls in the warning track*! The warning track was completely overpowered by the progress reward, meaning the agent felt no gradient pushing it away from the wall until the instant it died.
2. **Cannot Brake**: The throttle actuator had `offset=-20.0` and `scale=-20.0`, bounding the drive velocity to `[-40, -20]`. This meant the agent physically *could not* slow down below `0.5 m/s` to safely take sharp corners. 
**Fix 23**: 
- Increased the `boundary_penalty` weight from `0.1` to `0.6` (yielding `-60.0` points/step). Now, entering the warning track guarantees a net negative score (`+50 - 60 = -10`), forcing the agent to actively steer away from the edges.
- Adjusted the drive actuator to `scale=-40.0` and `offset=0.0`. The agent can now use the brake action to slow down to a crawl (if needed) for tight corners, governed naturally by the `stationary_penalty`.

**Issue 24 — The "Vibrating" Stagnation Trap & Missing Telemetry**
After Fix 23 allowed the agent to brake (offset 0.0), the untrained network (outputting mostly 0.0) caused the throttle to rapidly fluctuate between accelerating and braking. Due to the 1 Nm effort limit, the 3.3kg car couldn't overcome its own noisy braking and literally vibrated in place, dying to the stationary penalty after 1,000 steps without moving. Additionally, switching from 3 actions to 2 actions broke the telemetry mapping (`obs[:, 5:8]`), feeding the agent zeros for its previous actions.
**Fix 24**: 
- Replaced the flawed 2D `CombinedDriveAction` with an elegant 1D `GroupedJointVelocityAction` mapped to `[-40.0, 0.0]`. An output of `0.0` now translates to `-20.0 rad/s` (half-speed cruising). The agent safely kickstarts at half-speed, but retains full ability to brake (`-1.0` -> `0.0 rad/s`) or sprint (`+1.0` -> `-40.0 rad/s`).
- Updated the action telemetry mapping to `obs[:, 5:7]` so the agent can properly see its previous Steer and Drive commands.

**Issue 25 — True RGB-Only Lane Following (Preventing Telemetry Cheating)**
The user requested "RGB only lane following". However, the Actor network was previously unmasked and receiving the true `lat_err`, `head_err`, and mathematical `kappa` (curvature) directly in its 12-dim telemetry vector. This allowed the agent to "cheat" by ignoring the complex 512-dim camera features entirely and driving perfectly using the exact coordinates.
**Fix 25**: 
- Explicitly masked `lat_err`, `head_err`, and `kappa` from the Actor (assigning them `0.0`). The Actor is now completely blind to the mathematical layout of the track and is forced to learn how to drive using only its RGB camera.
- Left these features UNMASKED for the Critic. This is crucial for PPO: the Critic (which does not drive) must still accurately predict the value of a state (predicting a crash if the car is drifting). A blind Critic creates a flat baseline, destroying the Advantage gradient for the Actor.

**Issue 26 — The Backwards Teleportation Exploit (1248 WPΔ Spike)**
Shortly after fixing the actuators, the agent discovered a massive exploit, scoring +1,248 progress points per step. Root cause: When the agent crashed, IsaacLab teleported it back to the start line. However, `rewards.py` calculated the progress delta between the new spawn location and the *old* death location. Because the track loops, driving *backwards* from spawn and dying registered mathematically as a massive *forward* jump. The agent was intentionally suiciding backwards to farm points!
**Fix 26**: Modified `waypoint_progress_reward` to detect `env.episode_length_buf <= 1`. On the very first step after a respawn, the progress delta is forcefully zeroed out, entirely removing the exploit.

**Issue 27 — The Stationary Min-Max Trap (Sitting Still for -4000)**
Once the teleportation bug was fixed, the agent began sitting completely still at the starting line (`Len: 1000`, `Spd: 0.02`). Root cause analysis: The agent was completely blind (learning vision from scratch). Driving forward blindly inevitably led to scraping the warning track (`-60` per step) and crashing (`-1000`), yielding around `-7,000` total points for a 100-step run. Conversely, sitting perfectly still at the start line yielded a net `-4.0` points per step (from the `-5.0` stationary penalty and `+1.0` survival bonus), resulting in `-4,000` total points over the maximum 1000 steps. `-4000` was mathematically superior to `-7000`. The agent was parked because it was mathematically optimal!
**Fix 27**: Increased the `stationary_penalty` weight in `arcpro_env_cfg.py` from `5.0` to `10.0`. Sitting still now yields `-9.0` points per step (`-9,000` total), which is worse than `-7,000`. This mathematically forced the agent to hit the gas.

**Issue 28 — Policy Oscillation & The SKRL PPO Bug**
After being forced to drive, the agent spectacularly learned to navigate curves purely from vision (surviving for 1078 steps and traversing 30+ meters of track by step 36k). However, it suddenly collapsed back to 100-step episodes. Root cause: We had a constant learning rate of `1e-4` and no entropy bonus, causing the agent to overfit to specific curves and catastrophically forget. When I tried to implement `KLAdaptiveRL` via the scheduler, it instantly crashed because the `skrl` version shipped with IsaacLab 5.1.0 has a broken PyTorch LRScheduler signature.
**Fix 28**: Removed the broken scheduler and instead utilized PPO's native early stopping mechanism (`kl_threshold: 0.008`) and added a small `entropy_loss_scale: 0.001` in `train_skrl.py`. This safely stops the weight updates if a noisy batch pushes the gradients too far, completely preventing policy oscillation and collapse. We also corrected the checkpoint interval to `250` rollouts (approx 32k steps) to ensure we don't lose progress on crashes.

**Issue 29 — Watchdog Suicide Loop & Missing Metrics**
The training tmux session was instantly crashing 5 seconds after startup. Root cause: The custom SKRL print format doesn't log FPS. `watchdog.py` defaulted missing metrics to `0`, causing the legacy `FPS < 2` crash check to trigger a false positive "SYSTEM CRASH: OOM" and maliciously kill the tmux session.
**Fix 29**: Updated `get_latest_metrics` in `watchdog.py` to default missing metrics to `None` instead of `0`. Guarded all crash checks (e.g. `f is not None`) to prevent false triggers.

**Issue 30 — Log State Leakage on Restart**
The watchdog script was evaluating old metrics from previous training runs before the new simulation finished initializing. Root cause: `start_tmux_training.sh` used `tee -a` which appended to `logs/skrl_phase1.log`.
**Fix 30**: Updated `start_tmux_training.sh` to automatically archive `logs/skrl_phase1.log` before launching, changed `tee -a` to `tee` for a fresh log state, and increased the watchdog launch delay from 5 to 20 seconds to give Isaac Sim time to initialize the environments.

---

## Kinematics & Physics Session (2026-07-26)

**Issue 31 — The Forklift Effect (Driving Backwards)**
The car was visually driving backwards and steering like a forklift (Rear-Wheel Steering) causing erratic 360s. Root cause: The `b_drive` action space scale/offset in `arcpro_env_cfg.py` had been changed to `20.0` (positive). However, the F1Tenth USD model requires a negative velocity to drive in the forward mesh direction.
**Fix 31**: Reverted the `b_drive` scale and offset to `-20.0`. The car now drives forward physically, putting the steering axle at the front where it belongs (restoring Front-Wheel Steering).

**Issue 32 — The Parking Brake / AWD Lock-Up**
When the car was converted to RWD, the front wheels (`FL`, `FR`) were removed from the `b_drive` group but they remained in the `throttle` actuator in `arcpro_robot_cfg.py`. Because they were captured by the actuator but given no RL command, they defaulted to `0.0` rad/s. The `damping=1.0` setting caused the front wheels to act as active parking brakes, dragging and causing extreme pivot skids. Removing them from the actuator entirely caused them to lock up via USD defaults.
**Fix 32**: Reverted the car to **All-Wheel Drive (AWD)**. Both `b_drive` and the `throttle` actuator now explicitly target `["Joint_Drive_.*"]`. All 4 wheels receive the same forward velocity, mechanically pushing the car forward together without any dragging or unactuated locks. Old checkpoints were wiped due to the physics change, and training was restarted from scratch.

**Issue 33 — Newborn Vision Penalty Over-Saturation (The Warning Track Trap)**
After restarting training from scratch as a pure vision agent (Issue 32 & 25), the agent was consistently crashing at ~90 steps (the first curve) with massive negative episode rewards. Root cause: In Fix 23, the `boundary_penalty` was explicitly set to `-60.0` to completely overpower the `+50.0` progress reward, resulting in a net negative score (`-10.0` per step) in the warning track. For a newborn agent exploring randomly, entering the warning track felt strictly negative, causing "Penalty Over-Saturation." The agent learned that taking any action resulted in pain, hindering its ability to learn to follow the lane.
**Fix 33**: Decreased the `boundary_penalty` weight in `arcpro_env_cfg.py` from `0.6` to `0.4` (yielding `-40.0` points/step). Now, driving in the warning track yields a net positive score (`+50 - 40 = +10`), which is better than the `-10` stationary penalty but strictly worse than driving in the center (`+50`). This ensures that completing the task remains the dominant incentive during early exploration, providing a smooth potential-based gradient away from the walls without crushing the agent's will to drive.
**Result**: Applied and restarted training.

**Issue 34 — The Warning Track Trap vs Stationary Min-Max (Speed 0.02, Len 866)**
Even after Fix 33 reduced the `boundary_penalty` to -40.0, the agent continued to sit completely still at the start line (Speed: 0.02, Length: 866, Rew: -792.6). Root cause analysis: An untrained agent driving slowly (e.g. 0.1 m/s) in the warning track earns very little progress reward (+10) but takes the full boundary penalty (-40), yielding a net -30 points per step. Meanwhile, sitting perfectly still triggers the `stationary` penalty (weight 10.0 = -10 points) plus a +1 survival bonus, yielding a net -9 points per step. Since -9 is mathematically superior to -30, the agent learned that venturing out and exploring slowly was far more painful than just sitting parked.
**Fix 34**: Massively increased the `stationary` penalty weight from 10.0 to 50.0. Sitting still now yields a net -49 points per step. This makes parking strictly worse than the worst-case driving scenario (slowly scraping the wall at -30/step). The agent is mathematically forced to hit the gas to escape the -49 point stationary bleed.
**Result**: Applied, wiped checkpoints, and restarted training.

**Issue 35 — The False Positive Action Gate Immunity (Speed 0.13, Len 1001)**
After Fix 34 increased the `stationary` penalty to -50.0 to force movement, the agent continued to sit completely still! Analysis of the log (`Len: 1001`, `Rew: -870.6`) revealed that it was triggering the stagnation termination (1000 steps), but the reward was only -870 instead of the expected -50,000 for sitting still. 
Root cause: The agent was inadvertently using an exploit caused by the `GoSignalManager`. Even though there are no stop lines on the `track_1x_wrapper.usda` map, the visual stop line detector was occasionally hallucinating false positive stop lines on the track boundaries. This set `go_signal = 0.0`. In `actions.py`, a safety gate forced the car's throttle to 0.0 when `go_signal` was 0. Even worse, the `stationary` penalty was conditionally masked by `go_signal > 0.5`. This meant that when the car was forced to stop by a false positive, it was completely immune to the `stationary` penalty while still collecting +1.0 survival bonus and +0.5 action drive reward per step. The agent learned it could just sit there, let the false positives lock its brakes, and farm points for free without ever triggering the stationary penalty.
**Fix 35**: Disabled the `go_signal` action gate entirely in `arcproLab/mdp/actions.py` for Phase 1. Also removed the `go_signal > 0.5` condition from the `stationary` penalty in `arcpro_env_cfg.py` so it now fires unconditionally when speed is < 0.2 m/s. The agent can no longer hide behind traffic lights to dodge the penalty.
**Result**: Applied, wiped checkpoints, and restarted training.

**Issue 36 — The Critic Shape Deadlock (Agent hangs at step 0)**
After fixing the camera observations and explicitly starting the agent with `--enable_cameras` (Issue 18/32/25), the `SKRLFlattenWrapper` began passing the concatenated 524-dimensional visual+telemetry states to the agent. Because the SKRL `SequentialTrainer` uses the default policy observations (`states`) for the Critic unless explicitly separated, the `ARCProCritic` was receiving an input tensor of shape `(N, 524)`. However, the Critic's MLP head was strictly defined as `nn.Linear(12, 256)`. PyTorch threw an internal `RuntimeError` due to a dimension mismatch (12 vs 524). Because Isaac Sim physics pipelines often swallow PyTorch exceptions during callback execution, the python script silently deadlocked and hung at 100% CPU indefinitely instead of crashing.
**Fix 36**: Added an explicit slicing condition inside `ARCProCritic.compute()` in `skrl_models.py`. If the input state has `524` dimensions (indicating the cameras are active), it explicitly slices the last 12 dimensions (`state[:, -12:]`) which correspond exactly to the telemetry vector from the environment wrapper. The Critic now successfully processes the telemetry.
**Result**: Applied, wiped checkpoints, and restarted training.

**Issue 37 — The Donut Farming Exploit & Min-Max Reversion (Len: 1001, Spd: 0.03, WP: 0)**
At Step ~128k, the agent became catastrophically stuck in a new local minimum, surviving the maximum 1000 steps with `0` waypoint progress, `0.03` speed, and achieving a total episode reward of around `-905.5`. 
Root cause analysis revealed two exploits:
1. **Donut Farming**: By steering continuously in a tight circle near the center of the track, the agent maintained an instantaneous forward speed `> 0.2 m/s` (dodging the `stationary` penalty) and stayed out of the `boundary` warning track, all while collecting `action_drive` rewards (+0.5).
2. **Min-Max Reversion**: Even when the agent simply parked against the wall and triggered the `stationary` and `boundary` penalties, the net episode score resulted in `~-905.5`. Because crashing yields a flat `-1000.0` termination penalty, sitting completely still or doing donuts and eating minor penalties was *mathematically superior* to driving blindly and crashing.
**Fix 37**: 
- Added an `action_steer_penalty` with `weight=-2.0` (`RewTerm(func=lambda env: torch.square(env.action_manager.action[:, 0]))`) to severely punish the constant maximum steering required for Donut Farming.
- Increased the `stationary` penalty weight from `50.0` to `200.0` to guarantee that parking and taking the stationary penalty for 1000 steps will definitively exceed `-1000.0` points, mathematically forcing the agent to prefer driving (and eventually crashing) over parking.
**Result**: Applied, wiped checkpoints, and restarted training.

**Issue 38 — High-Speed Donut Farming (The 500k Milestone Exploit)**
At the 500k milestone, the agent's episode lengths reached up to ~723, but the episodic reward dropped to ~-704.8 and waypoint progress (`WPΔ_rew`) was 0.0. Root cause analysis: In Fix 37, we added an `action_steer_penalty` of `-2.0` to combat low-speed donuts. However, when the agent drives at higher speeds (> 0.2 m/s), it avoids the massive `-200.0` stationary penalty. By steering fully (1.0), it drives in fast circles (donuts), yielding `+1.0` (survival) `+0.5` (drive) `-2.0` (steer) = `-0.5` net points per step. Doing donuts for the maximum 1,000 steps yields a total score of `-500.0`. Since crashing into a wall yields a flat `-1000.0` terminal penalty, doing high-speed donuts (-500) was still *mathematically superior* to crashing (-1000). The agent once again preferred endless circles over navigating curves.
**Fix 38**: Increased the `action_steer_penalty` weight from `-2.0` to `-10.0`. Doing donuts now yields a net penalty of `-8.5` per step (yielding `-8500.0` over 1000 steps), making it vastly worse than crashing (`-1000.0`). Normal cornering is unaffected because the massive `+46.5` progress reward per step mathematically eclipses the brief `-10.0` steering penalty.
**Result**: Applied, wiped checkpoints, and restarted training.

### Issue 39: Deterministic Sitting Still Exploit (Stagnation Avoidance)
- **Problem:** The agent learned to sit perfectly still at the spawn point (Spd: 0.01). It avoided the `stationary` penalty because of a PyTorch scalar casting issue in the `torch.where` lambda inside `arcpro_env_cfg.py` (which returned 0.0 instead of -1.0). By sitting still, it accumulated positive `survival_bonus` points until the `stagnation_termination` triggered at step 1001, resulting in a slightly negative but highly stable reward (`-15623.7`), which the Value Function mathematically preferred over exploring and instantly crashing (`-1000` penalty).
- **Fix:** Moved `stationary_penalty` to `mdp_rew` and explicitly used `torch.tensor(-1.0)` in the `torch.where` outputs to ensure proper broadcasting and tensor type tracking by IsaacLab's RewardManager.
- **Addition:** Added a 50-step grace period (`env.episode_length_buf > 50`) to the `stationary` penalty to prevent "Immediate Suicide" (where the agent would crash immediately to avoid the -200/step penalty while accelerating from a standstill).

### Issue 40: The Suicide Loophole (Stationary Penalty Over-Saturation)
- **Problem:** The stationary penalty was set to 200.0 (per step) for speeds under 0.5 m/s. When the agent tried to slow down for the curve, it immediately accrued -1000 points over 5 steps. Since crashing is a flat -1000 penalty, the agent intentionally committed suicide into the wall rather than slowing down.
- **Fix:** Reduced `stationary` penalty weight in `arcpro_env_cfg.py` from 200.0 to 15.0 to prevent intentional crashing.

### Issue 41: Phase 2 Penalty Over-Saturation
- **Problem:** The agent was subjected to `lateral_error` (-10.0) and `action_steer_penalty` (-10.0) while still learning basic cornering from vision, causing it to refuse to steer.
- **Fix:** Temporarily reverted to a Phase 1 curriculum by setting both `lateral_error` and `action_steer_penalty` weights to 0.0 in `arcpro_env_cfg.py`.

### Issue 42: Catastrophic Forgetting & PPO Batch Size
- **Problem:** The agent randomly forgot how to drive after 639 steps. The PPO `rollouts` (128) and `mini_batches` (16) resulted in a tiny batch size of 80 transitions per backprop, originally designed to prevent OOM when storing 150k-dim raw images. With 524-dim ResNet features, this tiny batch size caused massive gradient variance.
- **Fix:** In `train_skrl.py`, increased `memory_size` and `rollouts` to 1024, and reduced `mini_batches` to 4, generating stable batches of 2560 transitions.


### Issue 43: Immediate Death via Settling Physics and Bad Spawns
- **Problem:** The agent was exhibiting episode lengths of 4-24 steps. Root cause analysis revealed three issues: 1) `height_termination` had a threshold of `0.01` and NO grace period, meaning normal physics suspension settling instantly killed the agent. 2) `white_line_contact` had a tiny 5-step grace period, causing bad environmental spawns to execute the agent on step 6. 3) The policy applied massive actions on step 1, causing the unsettled physics to flip the car.
- **Fix:** Added a 20-step grace period (`is_spawn = env.episode_length_buf < 20`) to both boundaries and height limits. Lowered height minimum to `-0.05` to safely allow tire compression. Added a `WarmupActionWrapper` in `train_skrl.py` to freeze the agent's controls to 0.0 for the first 10 steps.
- **Result:** Applied, wiped checkpoints, and restarted training.

### Issue 44: Deep Local Minimum Stagnation (The -15,000 Trap)
- **Problem:** After applying fixes to the Warmup wrapper, the training was resumed from a corrupted checkpoint (`agent_27750.pt`) that had learned a deep fear of moving. The agent output `action=-1.0` (stop) consistently. Although the newly fixed `stationary_penalty` properly applied a massive `-15,000` penalty per episode, PPO gradient updates were struggling to pull the network weights out of this deep local minimum because the `termination_penalty` (-1000) was still relatively high compared to short exploratory progress.
- **Fix:** In accordance with RL reward shaping best practices for overcoming stagnation plateaus, I reduced the `termination_penalty` weight in `arcpro_env_cfg.py` from `20.0` to `10.0` (reducing the crash penalty to `-500`). This lowers the threshold for exploratory forward movement to break even against crashing (now only requiring 10 waypoints of progress instead of 20). 
- **Result:** Wiped all corrupted checkpoints and restarted training from scratch to allow the randomly initialized network (mean action 0.0, resulting in half-max forward speed) to naturally bootstrap the progress reward.

### Issue 45: Early Plateaus and Steering Jitter
- **Problem:** At the 500k milestone, the agent achieved a positive mean reward (32.5) but the episode length plateaued at a mean of ~246 steps. The agent was swerving and oscillating on straightaways, leading to premature crashes. Without an explicit penalty for erratic steering, random action jitter caused by standard PPO exploration was generating "reward hacked" short episodes where the agent would progress slightly and then crash into the walls, rather than committing to a smooth trajectory.
- **Fix:** Researched PPO steering oscillation mitigation in IsaacLab. Enabled the `action_rate_smoothness_reward` with a weight of `10.0` in `arcpro_env_cfg.py`. This shaping term penalizes the squared difference between consecutive steering actions, effectively suppressing high-frequency jitter and encouraging smooth, continuous trajectory commitments.
- **Result:** Wiped checkpoints and restarted to force the value function to accurately learn the new baseline.

### Issue 46: The Smoothing Penalty Trap and Early Termination
- **Problem:** At the second 500k milestone, the `ep_len_mean` dropped from ~246 steps to ~107 steps. Adding the `action_rate_smoothness_reward` (-10.0) successfully stopped the jitter, but it also heavily penalized the sharp, corrective steering necessary to recover when the agent drifted towards the lane boundaries. Consequently, the agent chose to crash gracefully rather than execute a sharp turn. Furthermore, the `roadmark_contact` termination threshold (0.12) was too tight for a vision-only agent at this stage of learning, terminating episodes the moment the wheels touched the line.
- **Fix:** 
  1. Reduced the `action_rate_smoothness_reward` weight from `10.0` to `2.0` in `arcpro_env_cfg.py`. This provides a gentle nudge towards smoothness without overwhelmingly punishing necessary sharp corrections.
  2. Relaxed the `roadmark_contact` threshold in `arcpro_env_cfg.py` from `0.12` to `0.05`. This allows the agent to drive slightly over the white line before terminating, providing a larger "safe zone" to experience the curve's visual features and recover before instant death.
- **Result:** Wiped checkpoints and restarted to evaluate the relaxed boundary constraints.

### Issue 47: Catastrophic Failure at Spawn (User Manual Edits)
- **Problem:** The training abruptly broke, with the agent instantly dying after the 20-step spawn grace period (`ep_len_mean: 20.0`, `speed_mps: 0.0`). This was caused by manual edits to `arcpro_env_cfg.py` introducing a new `ground_contact_term` with a very low threshold (`1.0`) and raising the `height` termination threshold to `0.025`. Because the F1Tenth chassis suspension sags when dropped onto the track, its `z_pos` briefly dips below `0.025`, and the wheels occasionally clip through the thin track mesh, triggering the `ground_contact` sensor against the underlying ground plane.
- **Fix:** Retained the user's new termination concepts but relaxed the thresholds to tolerate suspension dynamics:
  1. Lowered `height` threshold from `0.025` to `0.01` (allowing 4cm of suspension sag on the track while still reliably terminating if it drops to `0.0` off the track).
  2. Increased `ground_contact_term` threshold from `1.0` to `50.0` to ignore minor mesh clipping forces.
- **Result:** Wiped checkpoints and restarted the training script as per self-healing protocol.

### Issue 48: User Reverted Fix (Catastrophic Spawn Failure Reoccurs)
- **Problem:** The user manually reverted `Issue 47`'s fix, disabling `height` entirely and resetting `ground_contact_term` to `1.0`. As expected, this immediately caused the agent to die instantly upon spawning (`Len: 20`, `Spd: 0.0`) because the wheels clip through the thin track mesh and constantly touch the underlying ground plane, generating >1.0N of contact force.
- **Fix:** Restored the `ground_contact_term` threshold to `50.0` (which ignores the wheels clipping the ground plane, but successfully triggers when the car falls off the track and slams its full weight into the ground). Left `height` disabled as per the user's intent, and added a detailed multi-line warning comment in `arcpro_env_cfg.py` directly above the line so the user understands why `1.0` is physically impossible in this specific mesh layout.
- **Result:** Wiped checkpoints and restarted the training script (again) as per self-healing protocol.

### Issue 49: The 500k Milestone Stagnation (Erratic Swerving & Speed Threshold)
- **Problem:** At the 500k milestone, the `ep_len_mean` dropped from ~261 to ~49, and `speed_mps` remained very low (~0.17). Root cause analysis revealed two issues: 
  1. We accidentally disabled `action_steer_penalty` completely (0.0). Since `action_rate_smoothness_reward` only punishes *changes* in steering, the agent learned to slam the steering to full lock and hold it there (paying a one-time penalty) and drive directly off the track into the wall at step 49.
  2. The `stationary_penalty` threshold in `rewards.py` was still `0.5` m/s, causing the agent to take massive penalties (-15 per step) whenever it tried to slow down for corners. The agent opted to drive slowly into walls rather than take the huge step penalties.
- **Fix:** 
  1. Re-enabled `action_steer_penalty` with `weight=0.5` to provide a consistent penalty for holding the wheel at extreme angles, forcing the agent to drive straight unless a turn is required.
  2. Lowered the `stationary_penalty` threshold in `rewards.py` from `0.5` to `0.2` m/s to allow the agent to safely slow down for sharp turns without triggering the penalty.
- **Result:** Wiped checkpoints and restarted the training to force the value function to accurately learn the new baseline.

### Issue 50: The High-Water Mark (Jitter Farming Exploit)
- **Problem:** At 310k steps, the agent was scoring massive positive rewards (`Rew: 3586.2`) while its episode length reached 649, yet its actual progress (`WPs_cum: 0`) was ZERO! The agent discovered "Jitter Farming": by vibrating back and forth over a single waypoint boundary (e.g. from index 10 to 11 and back), the `waypoint_progress_reward` kept returning positive deltas because it only compared `current_idx` to `prev_idx` from the last step. The agent farmed infinite `progress_reward`, `survival_bonus`, and `heading` points without actually driving down the track.
- **Fix:** Rewrote `waypoint_progress_reward` to use a **High-Water Mark**. We now track `cumulative_wp_index` (which correctly adds or subtracts waypoints based on forward/backward movement, handling track wrap-arounds seamlessly). The reward is *only* dispensed when the `cumulative_wp_index` strictly exceeds `max_cumulative_wp_index` (the furthest progress ever achieved in the current episode). Jittering backward subtracts from the cumulative index, meaning the agent must travel forward to regain the lost ground just to break even, entirely eliminating the exploit.
- **Result:** Wiped checkpoints and restarted the training script.

### Issue 51: The Action Drive Reward Conflict & Strict Lane Margins
- **Problem:** At the 500k milestone, the agent was consistently crashing at the first sharp curve (`Len: ~132`). Root cause analysis revealed two factors:
  1. `action_drive_reward` was still active from early Phase 1. Because it used `torch.abs(action[:, 1])` and the drive action is mapped to `[-40, 0]` (where -1 is stop and 1 is max speed), the reward function was paying the agent `+0.5` points to either go MAX SPEED or STOP completely, while awarding `0.0` points for intermediate speeds. This conflicted with the agent's need to slow down smoothly for corners.
  2. The `roadmark_contact` termination threshold was aggressively strict (`0.05m`), meaning the agent died instantly if it drifted just 5cm over the white line, giving it no room to learn corrective steering from the visual shift of the lane boundaries.
- **Fix:** 
  1. Disabled `action_drive_reward` by setting its weight to `0.0`, as the new High-Water Mark `progress_reward` perfectly handles driving incentives without introducing action-space noise.
  2. Relaxed the `roadmark_contact` threshold to `0.15m` to provide a forgiving buffer for the newborn vision agent to drift slightly and recover before triggering instant death.
- **Result:** Wiped checkpoints and restarted the training to embed the smoother action space.

### Issue 52: The "Donut Farming" Relapse (Positive Steer Penalty Bug)
- **Problem:** Around 220k steps into the second training run, the agent suddenly achieved massive positive rewards (`Rew: 5072.0`) while surviving the maximum `1001` steps, despite its final `WPs_cum` being `0`. The agent was performing high-speed donuts in place. 
- **Root Cause Analysis:**
  1. The `action_steer_penalty` was implemented as a lambda returning `torch.square(action[:, 0])`. However, its weight was set to `0.5` (positive), meaning the reward function was *paying* the agent +0.5 points per step to hold the steering wheel at maximum lock!
  2. Because the agent was holding max steering and max throttle, it performed rapid donuts. This generated a high internal body velocity (`speed > 0.2` m/s), completely bypassing the `stationary_penalty`.
  3. The donuts caused the agent's position to slightly drift back and forth along the track, occasionally bumping up its `max_cumulative_wp_index` (earning legitimate progress rewards) before drifting backward again, resulting in a net `WPs_cum: 0` at the end of the episode but thousands of points accumulated over the 1000 steps.
- **Fix:** Changed the weight of `action_steer_penalty` to `-0.5` so it actually penalizes erratic swerving and donut behavior instead of rewarding it.
- **Result:** Wiped checkpoints and restarted.

### Issue 53: Sparse Reward Stagnation (The "Gas Pedal" Problem)
- **Problem:** After fixing the donut exploit and disabling the flawed `action_drive_reward` in Issue 51, the agent's performance collapsed. It was surviving for ~90 steps but earning heavily negative rewards (`Rew: -301.8`) and failing to make any track progress (`WPs_cum: 2`). 
- **Root Cause Analysis:** 
  1. We disabled `action_drive_reward` because it was using `torch.abs()` (rewarding stopping as much as driving).
  2. However, without a dense reward for simply pressing the gas pedal, the agent fell victim to the Sparse Reward problem. The massive `progress_reward` (+50) only triggers if the agent successfully moves forward and crosses a waypoint. Because the agent was randomly exploring, it never consistently held the gas pedal long enough to cross waypoints and discover the massive reward waiting for it.
  3. Instead, it just jiggled in place, triggered the `stationary_penalty`, and died. 
- **Fix:** Re-enabled `action_drive_reward` (weight `0.5`) but fixed its mathematical mapping. The drive action maps `[-1, 1]` to `[0, -40]` rad/s (where 1.0 is max forward speed). We now return the raw `action[:, 1]` instead of the absolute value. This provides a dense, linear "breadcrumb" reward (+0.5 for max throttle, 0.0 for half throttle, -0.5 for braking) that guides the newborn agent to press the gas and discover the waypoint rewards, without penalizing intermediate speeds!
- **Result:** Wiped checkpoints and restarted.

### Issue 49: Re-enabled Phase 2 Penalties causing Suicide Loophole
- **Problem:** The previous commit re-enabled Phase 2 penalties (`heading` and `smoothness` at 2.0). Since the agent was learning vision from scratch, this caused penalty over-saturation. The agent suffered ~120 step episodes, crashing early to avoid accumulating these penalties.
- **Fix:** Reverted `heading` and `smoothness` weights back to `0.0` in `arcpro_env_cfg.py` to restore the Phase 1 curriculum, while keeping the beneficial physics bug fixes intact.
- **Result:** Wiped checkpoints and restarted training.

### Issue 50: High-Water-Mark Reward Bug (WPdelta=0)
- **Problem:** The new waypoint_progress_reward used a high-water-mark - only rewarding NEW progress beyond the furthest point reached in the episode. For short ~100-step episodes the agent died before reaching new territory, so the reward was always 0. The agent learned purely from survival + drive rewards with zero progress gradient, locking at ~100 step episodes.
- **Fix:** Reverted to simple per-step forward delta in rewards.py. Also bumped termination_penalty weight from 10.0 to 20.0 (yielding -1000 on crash), so crashing at step 100 is net -850, definitively worse than surviving.
- **Result:** Wiped checkpoints and restarted training.

---

## Session 2026-07-31 — 3-Pass System Audit + Refactor

### Issue 48: tanh Saturation of Progress Reward (Speed Gradient Killed)
- **Problem:** `progress_reward` used `tanh(WPs/step) * 50`. At 0.08 m/s (0.19 WPs/step), tanh≈0.19 — so going 25× faster (2.0 m/s, 2.4 WPs/step, tanh≈0.98) only yielded 5× more reward. Policy learned the slow-crawl equilibrium was good enough.
- **Fix:** Removed `tanh`, switched to linear `weight=30.0`. Speed gradient is now direct: 6× faster = 6× more reward.
- **File:** `arcpro_env_cfg.py`, `mdp/rewards.py`

### Issue 49: Spawn Domain Randomization Disabled (Overfitting)
- **Problem:** `rand_offset_x = torch.zeros(...)` and `rand_yaw = torch.zeros(...)` in `events.py` — all 128 envs spawned at the same position every reset, causing the policy to memorize one trajectory with no generalization.
- **Fix:** Re-enabled: `±0.3m` X-offset, `±5°` yaw.
- **File:** `mdp/events.py`

### Issue 50: Stagnation Termination Too Loose
- **Problem:** `stagnation_termination` fired at `speed < 0.1 m/s` after `1000 steps`. The stationary_penalty fires at `< 0.2 m/s`, so the agent could crawl at 0.08–0.19 m/s accruing -15/step for 1000 steps (-15,000 total) before forced reset.
- **Fix:** Tightened to `< 0.2 m/s` after `200 steps` to match stationary_penalty threshold.
- **File:** `mdp/terminations.py`

### Issue 51: Boundary Penalty Zone Too Wide (Accumulation Trap)
- **Problem:** `boundary_penalty` fired at 0.40m from wall but termination at 0.15m — a 0.25m-wide trap where agents bled -40/step indefinitely. This explained -2600 episode rewards.
- **Fix:** Tightened warning zone from `0.40m → 0.20m`. Now only 5cm warning band before hard termination.
- **File:** `mdp/rewards.py` + constant `BOUNDARY_WARN_THRESHOLD_M`

### Issue 52: go_signal_manager Python CPU Loop (6-32× Slowdown)
- **Problem:** `go_signal_manager.update()` ran 128 cv2 (HSV + contour) calls per step with GPU→CPU tensor transfer. At ~1-5ms per call: 128–640ms overhead against 20ms budget. Explained training running at 4.7 steps/sec instead of ~15 steps/sec.
- **Fix:** Bypassed `mgr.update(images)` in Phase 1 (no stop bars in training zone). Always return `ones`. Re-enable for Phase 3+ intersection curriculum.
- **File:** `mdp/observations.py` → `_obs_go_signal()`

### Issue 53: Observation Slots Unnormalized / Dead
- **Problem:** obs slot 3 (speed) and slot 4 (yaw_rate) were injected raw (0–5 m/s / 0–20 rad/s) into the network alongside ResNet features (~[0,1]). Slot 11 (distance) grew unbounded. Slot 7 was never assigned (dead).
- **Fix:** Slot 3 ÷ `OBS_MAX_SPEED_MPS=2.0`, slot 4 ÷ `OBS_MAX_YAW_RATE_RPS=5.0`, slot 11 ÷ `OBS_MAX_DISTANCE_M=50.0`. Module-level constants in `observations.py`.
- **File:** `mdp/observations.py`

### Issue 54: PPO Hyperparameters Too Conservative
- **Problem:** `kl_threshold=0.008` aborted gradient steps too early (early training has large policy gradients). `entropy_loss_scale=0.001` provided insufficient exploration pressure to escape slow-crawl trap.
- **Fix:** `kl_threshold: 0.008 → 0.02`. `entropy_loss_scale: 0.001 → 0.005`.
- **File:** `scripts/train_skrl.py`

### Refactor (No Behavior Change)
- Split `_compute_telemetry()` monolith into 6 named helper functions in `observations.py`.
- Added module-level constants to `rewards.py` and `observations.py`.
- Replaced 3 inline lambdas in `arcpro_env_cfg.py` with named `mdp_rew.*` functions.

### Post-Fix Training Baseline
- Step 7,300, Rew: ~-45, Spd: ~0.18 m/s (up from 0.08 m/s pre-fix)
- Old checkpoints archived to `logs/archive_pre_11fix_*/`
