# GSD State

**Active Milestone:** M001: Milestone 3: HD Perception and Production Hardening
**Active Slice:** S01: S01
**Phase:** evaluating-gates
**Requirements Status:** 5 active · 0 validated · 0 deferred · 0 out of scope

## Milestone Registry
- 🔄 **M001:** Milestone 3: HD Perception and Production Hardening

## Recent Decisions
- **D008:** Resolved environment physics/camera orientation bugs (camera native backward offset vs drive direction) and pushed cleanup to `dev` branch.
- **D009:** Fixed 5 critical RL bugs (Suicide Bias, 3D Distance Z-Axis Boundary Bug, Distance Inflation, Double Stepping FSM, 1-Step Reward Delay).
- **D010:** (REVERTED) Injected 5 forward-looking waypoints into the telemetry vector. This was reverted because providing numerical anticipation completely bypassed the requirement (R002) for the agent to learn HD Vision-based navigation.
- **D011:** (SUPERSEDED) Implemented Balanced Transfer Learning in `fusion_policy.py`. We unfroze `layer4` because the agent seemed blind when the network was fully frozen. However, this still caused late-stage KL divergence spikes.
- **D012:** Discovered the TRUE root cause of the vision failure: the CNN normalization block in `fusion_policy.py` was bypassing ImageNet `mean`/`std` normalization for `[0, 1]` images. This fed unnormalized tensors into frozen ImageNet layers, producing garbage feature maps (rendering the agent blind). Fixed the normalization math and completely FROZE the entire ResNet-18 backbone again. The agent can now "see" the track using stable, frozen edge-detectors, and PPO is 100% stable.
- **D013:** Fixed the "Zoolander Bug". `ActionCfg` fields were sorted alphabetically by IsaacLab (`drive`, `steering`), meaning the `train_policy.py` bounds `[Steering, Throttle, Brake]` applied the brake's `[0.0, 1.0]` bound to the steering action, physically preventing the robot from turning right. Renamed fields to `a_steering` and `b_drive` to restore the correct order. Also reduced the stationary penalty from 1.0 to 0.05 to prevent the agent from intentionally crashing to avoid massive stagnation bleed.
- **D014:** Fixed "Reward Farming Exploit" & "Stagnation Bias". The agent refused to learn to turn at 2M steps, opting to drive slowly straight into the wall at 0.37 m/s. Discovered that the Quadratic Target Velocity reward and Lane Centering reward granted massive positive step-rewards just for existing. The slower it drove, the more steps it stayed alive, accumulating more reward before the crash. 
  - Overhauled `rewards.py`: `progress_reward` is now strictly linear `forward_speed` (Total Reward = Distance Traveled, negating the time exploit).
  - `lateral_error_reward` is now a pure penalty.
  - **Stagnation Bias Fix:** The agent then tried to stand completely still because the `-200` crash penalty wasn't being correctly applied to timeouts. Fixed `termination_penalty` to use IsaacLab's `env.reset_terminated` instead of `env.reset_buf`. 
  - **Progress Weight:** Increased `progress_reward` weight to 5.0x so driving forward into the wall is mathematically more profitable than standing still, bootstrapping movement.
- **D015:** Fixed "Playing Dead Trap". After the distance reward fix, the agent became so terrified of the `-200` crash penalty that it refused to move, choosing to take a `-50` stagnation penalty over a `-200` wall collision. Disabled `termination_penalty` entirely. Since the reward is now strictly physical distance traveled, crashing naturally stops point accumulation, making the crash penalty redundant and actively harmful to exploration.
- **D016:** Restored environment testing GUI (`debug.sh`) and patched `verify_policy.py` to support the new nested `"policy"` observation structure. Hardened `watchdog.py` self-healing sequence to launch autonomous repair agents asynchronously, preventing self-termination. Replaced memory-heavy subagents with lightweight daemon-based Antigravity summons for 250k milestone reporting.

## Blockers
- None

## Next Action
- A new SB3 training session is currently running in `tmux` (Phase 1 Curriculum).
- The `watchdog.py` daemon is monitoring for crashes and will autonomously summon an Antigravity agent to repair the environment if divergence/stagnation occurs.
- The `watchdog.py` daemon will also autonomously ping the user whenever the agent crosses a 250k timestep milestone.
- **Incoming Agent:** Simply wait for milestone reports from the watchdog, or step in if a repair agent flags an issue it cannot resolve.
