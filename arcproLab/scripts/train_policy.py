# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train a Hierarchical SB3 policy for ARCPro Lane Following.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of parallel simulation environments.")
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument("--total_timesteps", type=int, default=5000000, help="Total timesteps to train (5M recommended for HPPP).")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to a checkpoint to resume from.")
parser.add_argument("--batch_size", type=int, default=64, help="PPO minibatch size.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import torch
import warnings
import numpy as np
import gymnasium as gym
import cv2
from datetime import datetime

# Silence gym warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

# Add both root and arcproLab to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
POLICY_STACK_DIR = os.path.abspath(os.path.join(ROOT_DIR, "..", "arc_rl_isacc_policy"))

if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)
if POLICY_STACK_DIR not in sys.path:
    sys.path.insert(0, POLICY_STACK_DIR)

from sb3_contrib import RecurrentPPO
from stable_baselines3.common.callbacks import CheckpointCallback, BaseCallback
from stable_baselines3.common.vec_env import VecNormalize, VecEnv, VecMonitor

from isaaclab.envs import ManagerBasedRLEnv
from arcpro_env_cfg import ARCProEnvCfg

# Submodule Imports
from policies.hierarchical_policy import HierarchicalPathPlanningPolicy
from policies.fusion_policy import FusionFeaturesExtractor
from wrappers.waypoint_tracking_wrapper import WaypointTrackingWrapper

class HPPPDirectBridge(VecEnv):
    """
    Bridge between Isaac Lab and HPPP Brain.
    Acts as a Stable Baselines3 VecEnv and preserves the Dict observation space.
    """
    def __init__(self, env: gym.Env):
        self.venv = env
        self.num_envs = env.get_wrapper_attr("num_envs")
        self.device = env.get_wrapper_attr("device")
        
        # Note: In modern Isaac Lab, groups with single terms are already Boxes
        single_obs_space = env.get_wrapper_attr("single_observation_space")
        img_space = single_obs_space.spaces["visual"]
        
        # Mastery: Use uint8 for storage to save 4x VRAM in the rollout buffer
        transposed_img_space = gym.spaces.Box(
            low=0, high=255, 
            shape=(img_space.shape[2], img_space.shape[0], img_space.shape[1]),
            dtype=np.uint8
        )
        observation_space = gym.spaces.Dict({
            "vec": single_obs_space.spaces["policy"],
            "image": transposed_img_space
        })
        
        # Override Isaac Lab's unbounded ActionManager space with strict SB3 bounds
        low = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        high = np.array([1.0, 1.0, 1.0], dtype=np.float32)
        action_space = gym.spaces.Box(low=low, high=high, dtype=np.float32)
        
        super().__init__(self.num_envs, observation_space, action_space)
        
        self.render_mode = None
        self.metadata = {"render_modes": []}
        self.prev_action = torch.zeros((self.num_envs, 3), device=self.device)

    def reset(self):
        obs_dict, info = self.venv.reset()
        self.prev_action.zero_()
        return self._process_obs(obs_dict)

    def step_async(self, actions):
        self._async_actions = actions

    def step_wait(self):
        actions_torch = torch.from_numpy(self._async_actions).to(self.device)
        
        # Store prev_action for the reward function (in env.extras)
        self.venv.extras["prev_action"] = self.prev_action.clone()
        
        obs_dict, rewards, terminated, truncated, info = self.venv.step(actions_torch)
        dones = (terminated | truncated).cpu().numpy()
        
        # Update prev_action for next step
        self.prev_action.copy_(actions_torch)

        # Extraction for custom logging (Mastery v2.6)
        # We extract speed and boundary distances for fine-grained debugging
        robot_asset = self.venv.scene["robot"]
        # Use absolute magnitude for logging to ensure positive values during Control Flip
        self.venv.extras["speed"] = torch.norm(robot_asset.data.root_lin_vel_b[:, :2], dim=1).clone()
        
        # SB3 expects info to be a list of dicts, one for each env
        infos = [{} for _ in range(self.num_envs)]
        for key, value in info.items():
            if isinstance(value, torch.Tensor) and value.shape[0] == self.num_envs:
                val_np = value.cpu().numpy()
                for i in range(self.num_envs):
                    infos[i][key] = val_np[i]
            else:
                for i in range(self.num_envs):
                    infos[i][key] = value

        # Manually add the extra telemetry to the info dicts
        dist_y = self.venv.extras.get("dist_y", torch.zeros(self.num_envs)).cpu().numpy()
        dist_w = self.venv.extras.get("dist_w", torch.zeros(self.num_envs)).cpu().numpy()
        speed = self.venv.extras["speed"].cpu().numpy()

        # go_signal telemetry (Path A diagnostic instrumentation).
        # We pull both the published gate value and the underlying FSM state so
        # we can tell apart "policy crawls because gate held it down" from
        # "policy crawls anyway." If the manager isn't initialized (e.g. cameras
        # off) we fill zeros; logging will show that as gate-always-open.
        go_signal_np = self.venv.extras.get(
            "go_signal", torch.ones(self.num_envs, device=self.device)
        ).cpu().numpy()
        try:
            from mdp.go_signal_manager import get_go_signal_manager
            gsm = get_go_signal_manager(self.num_envs, str(self.device))
            go_state_np = gsm.state.cpu().numpy()
            stop_bar_dist_np = gsm.last_distance.cpu().numpy()
            stop_bar_det_np = gsm.last_detected.cpu().numpy().astype(np.float32)
        except Exception:
            go_state_np = np.zeros(self.num_envs, dtype=np.int64)
            stop_bar_dist_np = np.full(self.num_envs, float("nan"), dtype=np.float32)
            stop_bar_det_np = np.zeros(self.num_envs, dtype=np.float32)

        # Per-termination flags + episode length at done. We read directly off
        # the termination manager's per-term buffer so we can attribute each
        # reset to a specific cause (boundary vs gate-hit vs stagnation etc.).
        term_flags = {}
        try:
            tm_mgr = self.venv.termination_manager
            for term_name in tm_mgr.active_terms:
                term_flags[term_name] = tm_mgr.get_term(term_name).cpu().numpy()
        except Exception:
            pass
        ep_len_buf = self.venv.episode_length_buf.cpu().numpy()

        for i in range(self.num_envs):
            infos[i]["dist_yellow"] = dist_y[i]
            infos[i]["dist_white"] = dist_w[i]
            infos[i]["speed"] = speed[i]
            infos[i]["go_signal"] = float(go_signal_np[i])
            infos[i]["go_state"] = int(go_state_np[i])
            infos[i]["stop_bar_dist"] = float(stop_bar_dist_np[i])
            infos[i]["stop_bar_detected"] = float(stop_bar_det_np[i])
            # When this step ended the episode, record which terms fired and the length.
            if dones[i]:
                infos[i]["ep_len_at_done"] = int(ep_len_buf[i])
                for term_name, flags in term_flags.items():
                    if bool(flags[i]):
                        infos[i][f"term_{term_name}"] = 1.0

        return self._process_obs(obs_dict), rewards.cpu().numpy(), dones, infos

    def _process_obs(self, obs_dict):
        # Isaac Lab returns normalized float32 (0-1) if "normalize": True is set in cfg
        # We convert to uint8 (0-255) for storage efficiency
        img = obs_dict["visual"].cpu().numpy()
        img_uint8 = (np.clip(img, 0, 1) * 255).astype(np.uint8)
        
        # Transpose from (B, H, W, C) to (B, C, H, W)
        img_transposed = np.transpose(img_uint8, (0, 3, 1, 2))
        return {
            "vec": obs_dict["policy"].cpu().numpy(),
            "image": img_transposed
        }

    def close(self):
        self.venv.close()

    def get_attr(self, attr_name, indices=None):
        val = getattr(self.venv, attr_name, None)
        return [val] * self.num_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self.venv, attr_name, value)

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        method = getattr(self.venv, method_name)
        val = method(*method_args, **method_kwargs)
        return [val] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    @property
    def base_env(self):
        return self.venv

def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True 
    env_cfg.__post_init__() 

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # 3. Wrap for Hierarchical Tracking (Required for auxiliary loss)
    env = WaypointTrackingWrapper(env)
    
    # 4. Bridge to SB3 and HPPP Contract
    env = HPPPDirectBridge(env)
    
    # 4.5. Monitor Episode Statistics
    env = VecMonitor(env)
    
    # 5. Add Normalization
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)

    # 6. Define Log Directory
    log_dir = os.path.join("logs", "ppo", datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)

    # 7. Define Policy & Model
    policy_kwargs = dict(
        features_extractor_class=FusionFeaturesExtractor,
        features_extractor_kwargs=dict(features_dim=256),
        lstm_hidden_size=256,
        n_lstm_layers=1,
        enable_critic_lstm=True,
        share_features_extractor=True,
        # Reduce initial action variance for continuous bounded control
        # Default is log_std=0.0 (std=1.0). -0.5 is std=0.6. Prevents instant boundary crashes.
        log_std_init=-0.5,
        # Hierarchical parameters
        num_waypoints=5,
        waypoint_horizon=2.5,
        use_kinematic_anchors=True,
    )

    if args_cli.checkpoint:
        print(f"Resuming from checkpoint: {args_cli.checkpoint}")
        checkpoint_dir = os.path.dirname(args_cli.checkpoint)
        norm_file = os.path.join(checkpoint_dir, "vec_normalize.pkl")
        if os.path.exists(norm_file):
            print(f"Loading normalization stats from: {norm_file}")
            env = VecNormalize.load(norm_file, env)
            env.training = True 
            
        model = RecurrentPPO.load(
            args_cli.checkpoint,
            env=env,
            device="cuda",
            custom_objects={
                "learning_rate": 1e-5,
                "target_kl": 0.01,
                "clip_range": 0.2,
                "ent_coef": 0.0,
                "tensorboard_log": os.path.join(log_dir, "tb")
            }
        )
    else:
        model = RecurrentPPO(
            HierarchicalPathPlanningPolicy,
            env,
            verbose=1,
            learning_rate=5e-5,
            n_steps=512,        # 4096 samples per update
            batch_size=args_cli.batch_size,
            n_epochs=4,         # Reduced from 10 to stabilize KL/clip explosions
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.0,       # Killed entropy bonus to stop forced random crashing
            target_kl=0.015,    # Circuit breaker to stop updates if KL blows up
            tensorboard_log=os.path.join(log_dir, "tb"),
            seed=args_cli.seed,
            device="cuda",
            policy_kwargs=policy_kwargs,
        )

    # 8. Callbacks
    class RewardLoggerCallback(BaseCallback):
        # Path A diagnostic instrumentation. Rolling sums reset on rollout
        # start so TB sees one accurate scalar per update instead of whichever
        # value `record()` happened to write last.
        def __init__(self, verbose: int = 0):
            super().__init__(verbose)
            self._reset_accumulators()

        def _reset_accumulators(self) -> None:
            self._n_samples = 0
            self._sum_speed = 0.0
            self._sum_dist_w = 0.0
            self._sum_dist_y = 0.0
            self._sum_go_signal = 0.0
            self._sum_in_stop = 0.0
            self._sum_in_depart = 0.0
            self._sum_bar_detected = 0.0
            self._sum_bar_dist = 0.0
            self._n_bar_detected = 0
            self._term_counts: dict[str, int] = {}
            self._n_done = 0
            self._sum_ep_len_at_done = 0.0

        def _on_rollout_start(self) -> None:
            self._reset_accumulators()

        def _on_step(self) -> bool:
            infos = self.locals.get("infos", [{}])
            dones = self.locals.get("dones", [False] * len(infos))
            n = len(infos)

            for i, info in enumerate(infos):
                self._sum_speed   += float(info.get("speed", 0.0))
                self._sum_dist_w  += float(info.get("dist_white", 0.0))
                self._sum_dist_y  += float(info.get("dist_yellow", 0.0))
                self._sum_go_signal += float(info.get("go_signal", 1.0))
                gs = int(info.get("go_state", 0))
                if gs == 1:
                    self._sum_in_stop += 1.0
                elif gs == 2:
                    self._sum_in_depart += 1.0
                if float(info.get("stop_bar_detected", 0.0)) > 0.5:
                    self._sum_bar_detected += 1.0
                    d = float(info.get("stop_bar_dist", float("nan")))
                    if d == d:  # NaN-safe
                        self._sum_bar_dist += d
                        self._n_bar_detected += 1

                if bool(dones[i]):
                    self._n_done += 1
                    self._sum_ep_len_at_done += float(info.get("ep_len_at_done", 0))
                    for key in info:
                        if key.startswith("term_"):
                            self._term_counts[key] = self._term_counts.get(key, 0) + 1
            self._n_samples += n

            if self.n_calls % 1000 == 0 and self._n_samples > 0:
                mean_speed = self._sum_speed / self._n_samples
                mean_dist_w = self._sum_dist_w / self._n_samples
                mean_dist_y = self._sum_dist_y / self._n_samples
                go_mean = self._sum_go_signal / self._n_samples
                stop_frac = self._sum_in_stop / self._n_samples
                if len(self.model.ep_info_buffer) > 0:
                    ep_rew = sum(e["r"] for e in self.model.ep_info_buffer) / len(self.model.ep_info_buffer)
                    ep_len = sum(e["l"] for e in self.model.ep_info_buffer) / len(self.model.ep_info_buffer)
                else:
                    ep_rew = 0.0
                    ep_len = 0.0
                print(
                    f"[PROGRESS] Step {self.n_calls} | EpRew: {ep_rew:.2f} | EpLen: {ep_len:.1f} "
                    f"| AvgSpeed: {mean_speed:.2f} m/s | DW: {mean_dist_w:.2f}m | DY: {mean_dist_y:.2f}m "
                    f"| GoMean: {go_mean:.2f} | StopFrac: {stop_frac:.2%}"
                )
                sys.stdout.flush()

            return True

        def _on_rollout_end(self) -> None:
            if self._n_samples == 0:
                return
            n = self._n_samples
            self.logger.record("rollout/speed_mps",   self._sum_speed   / n)
            self.logger.record("rollout/dist_white_m", self._sum_dist_w / n)
            self.logger.record("rollout/dist_yellow_m", self._sum_dist_y / n)
            self.logger.record("go_signal/mean",       self._sum_go_signal   / n)
            self.logger.record("go_signal/stop_frac",  self._sum_in_stop     / n)
            self.logger.record("go_signal/depart_frac", self._sum_in_depart  / n)
            self.logger.record("go_signal/bar_detected_frac", self._sum_bar_detected / n)
            if self._n_bar_detected > 0:
                self.logger.record("go_signal/bar_dist_when_detected_m",
                                   self._sum_bar_dist / self._n_bar_detected)
            # Termination attribution: count per cause and mean ep length at death.
            for term_key, count in self._term_counts.items():
                self.logger.record(f"terminations/{term_key}_count", count)
            self.logger.record("terminations/total_done", self._n_done)
            if self._n_done > 0:
                self.logger.record("terminations/ep_len_at_done_mean",
                                   self._sum_ep_len_at_done / self._n_done)

    class SaveVecNormalizeCallback(BaseCallback):
        def __init__(self, save_path: str, save_freq: int, verbose: int = 0):
            super().__init__(verbose)
            self.save_path = save_path
            self.save_freq = save_freq

        def _on_step(self) -> bool:
            if self.n_calls % self.save_freq == 0:
                self.training_env.save(os.path.join(self.save_path, "vec_normalize.pkl"))
            return True

    # save_freq is per vectorized step. 500k / num_envs = frequency.
    save_freq = max(1, 500000 // args_cli.num_envs)
    checkpoint_callback = CheckpointCallback(save_freq=save_freq, save_path=log_dir, name_prefix="model")
    vec_norm_callback = SaveVecNormalizeCallback(save_path=log_dir, save_freq=save_freq)
    reward_logger_callback = RewardLoggerCallback()

    # 9. Train
    print(f"Starting Hierarchical training for {args_cli.total_timesteps} steps...")
    model.learn(
        total_timesteps=args_cli.total_timesteps,
        callback=[checkpoint_callback, vec_norm_callback, reward_logger_callback],
        progress_bar=True
    )

    # 10. Save Final Model
    model.save(os.path.join(log_dir, "model_final"))
    env.save(os.path.join(log_dir, "vec_normalize.pkl"))
    
    # 11. Close
    env.close()

if __name__ == "__main__":
    main()
