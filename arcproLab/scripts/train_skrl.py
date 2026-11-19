import argparse
import os

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(
    description="Train SKRL AAC policy for ARCPro Lane Following."
)
parser.add_argument(
    "--num_envs",
    type=int,
    default=20,
    help="Number of parallel simulation environments.",
)
parser.add_argument("--seed", type=int, default=42, help="Seed for the environment.")
parser.add_argument(
    "--total_timesteps", type=int, default=5000000, help="Total timesteps to train."
)
parser.add_argument(
    "--resume", type=str, default="", help="Path to checkpoint to resume from."
)
parser.add_argument(
    "--fresh",
    action="store_true",
    default=False,
    help="Start fresh training (default when --resume is omitted; ignores any --resume path).",
)
parser.add_argument(
    "--warm_start_policy",
    type=str,
    default="",
    help="Optional checkpoint path to load ONLY policy weights while initializing Critic, value_preprocessor, and decoupled optimizers fresh.",
)
parser.add_argument(
    "--vision_mode",
    type=str,
    default="rgb",
    choices=["rgb", "yuv"],
    help="Camera preprocessing mode passed to SKRLFlattenWrapper ('rgb' default or 'yuv' fallback).",
)
parser.add_argument(
    "--initial_step",
    type=int,
    default=None,
    help="Initial step offset for cumulative step tracking.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import re
import subprocess
import sys
import warnings
from datetime import datetime

import torch

warnings.filterwarnings("ignore", category=DeprecationWarning)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

from agents.skrl_models import ARCProActor, ARCProCritic
from agents.skrl_wrappers import SKRLFlattenWrapper, WarmupActionWrapper
from agents.telemetry_ppo import TelemetryPPO
from arcpro_env_cfg import ARCProEnvCfg
from isaaclab.envs import ManagerBasedRLEnv
from skrl.envs.wrappers.torch import IsaacLabWrapper
from skrl.memories.torch import RandomMemory
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.trainers.torch import SequentialTrainer

try:
    from skrl.agents.torch.ppo import PPO_DEFAULT_CONFIG
except ImportError:
    from dataclasses import asdict
    from skrl.agents.torch.ppo import PPO_CFG

    PPO_DEFAULT_CONFIG = asdict(PPO_CFG())


def main():
    # 1. Setup Environment Configuration
    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.enable_cameras:
        env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    # 2. Create Environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    env = WarmupActionWrapper(env, warmup_steps=10)

    # 3. Wrap for SKRL
    # The IsaacLabWrapper automatically extracts the "critic" observation group
    # (if it exists) into env.state_space and env.state(), enabling AAC.
    env = IsaacLabWrapper(env)

    # 3.5 Flatten for SKRL RandomMemory limitation
    env = SKRLFlattenWrapper(
        env, vision_mode=args_cli.vision_mode, augment=True, training=True
    )

    # 4. Define Memory: 512 rollouts stored in Host System RAM (device="cpu")
    # Frees 100% of GPU replay buffer VRAM, allowing 20 parallel camera envs with zero OOM risk!
    memory = RandomMemory(memory_size=512, num_envs=env.num_envs, device="cpu")

    # 5. Define Models
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")

    # Use env.state_space for the Critic if available, fallback to observation_space
    state_space = getattr(env, "state_space", env.observation_space)
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

    # 6. Configure Agent
    cfg_ppo = PPO_DEFAULT_CONFIG.copy()
    cfg_ppo["rollouts"] = 512
    cfg_ppo["mini_batches"] = (
        16  # Mini-batch size = (512 * 20 / 16) = 640 transitions streamed per backprop
    )
    cfg_ppo["learning_rate"] = (
        3e-4  # Optimized learning rate for 21k SpatialSoftmax policy head
    )
    cfg_ppo["random_timesteps"] = 0
    cfg_ppo["learning_starts"] = 0
    cfg_ppo["state_preprocessor"] = None
    cfg_ppo["value_preprocessor"] = RunningStandardScaler
    cfg_ppo["value_preprocessor_kwargs"] = {"size": 1, "device": "cuda:0"}
    cfg_ppo["clip_predicted_values"] = True
    cfg_ppo["value_clip"] = 0.2
    cfg_ppo["value_loss_scale"] = 0.5
    cfg_ppo["mirror_symmetry_weight"] = 0.5
    # Use native PPO KL early stopping and Entropy to prevent policy collapse
    cfg_ppo["kl_threshold"] = (
        0.016  # Relaxed from 0.008 — too tight was causing snap-back collapses
    )
    cfg_ppo["entropy_loss_scale"] = (
        0.005  # Raised from 0.001 for more exploration out of local minima
    )
    cfg_ppo["learning_rate_scheduler"] = None

    # Branch-aware experiment directory to isolate checkpoints by branch
    try:
        branch_name = (
            subprocess.check_output(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"], stderr=subprocess.DEVNULL
            )
            .decode()
            .strip()
        )
        branch_slug = re.sub(r"[^a-zA-Z0-9_\-]", "_", branch_name)
    except Exception:
        branch_slug = "main"

    log_folder = "ppo_skrl" if branch_slug == "main" else f"ppo_skrl_{branch_slug}"
    log_dir = os.path.join("logs", log_folder, datetime.now().strftime("%Y%m%d-%H%M%S"))
    cfg_ppo["experiment"]["directory"] = log_dir
    cfg_ppo["experiment"]["write_interval"] = 100
    cfg_ppo["experiment"]["checkpoint_interval"] = (
        50000  # Save every 50k timesteps (~2.4 hours)
    )

    is_fresh = args_cli.fresh or not args_cli.resume
    # Auto-detect initial_step from resume checkpoint if not explicitly provided
    initial_step = args_cli.initial_step
    if initial_step is None and args_cli.resume and not is_fresh:
        match = re.search(r"agent_(\d+)", args_cli.resume)
        if match:
            initial_step = int(match.group(1))
            print(
                f"[Train] Auto-detected initial step offset from checkpoint: {initial_step}"
            )
        else:
            initial_step = 0
    elif initial_step is None:
        initial_step = 0

    agent = TelemetryPPO(
        models=models,
        memory=memory,
        cfg=cfg_ppo,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cuda:0",
        initial_step=initial_step,
        num_envs=args_cli.num_envs,
    )

    if args_cli.warm_start_policy:
        print(
            f"[Train] Warm-starting ONLY policy weights from: {args_cli.warm_start_policy} "
            "(Critic, value_preprocessor, and decoupled optimizers initialized fresh)"
        )
        ckpt = torch.load(
            args_cli.warm_start_policy, map_location="cuda:0", weights_only=False
        )
        policy_sd = (
            ckpt["policy"] if isinstance(ckpt, dict) and "policy" in ckpt else ckpt
        )
        missing, unexpected = agent.policy.load_state_dict(policy_sd, strict=False)
        print(
            f"[Train] Policy warm-start loaded (missing={len(missing)}, unexpected={len(unexpected)})."
        )
    elif not is_fresh and args_cli.resume:
        print(f"Resuming from checkpoint: {args_cli.resume}")
        agent.load(args_cli.resume)
        # Ensure configured learning rate overrides serialized optimizer state from old checkpoint
        for opt_name in ("policy_optimizer", "value_optimizer", "optimizer"):
            opt = getattr(agent, opt_name, None)
            if opt is not None:
                for param_group in opt.param_groups:
                    param_group["lr"] = cfg_ppo["learning_rate"]
        print(
            f"[Train] Explicitly updated optimizer learning rate to: {cfg_ppo['learning_rate']}"
        )
    else:
        print("[Train] Starting fresh training (policy, critic, value_preprocessor, and decoupled optimizers initialized fresh).")

    # 7. Train
    print(f"Starting SKRL AAC training for {args_cli.total_timesteps} steps...")
    trainer_cfg = {"timesteps": args_cli.total_timesteps, "disable_progressbar": True}
    trainer = SequentialTrainer(cfg=trainer_cfg, env=env, agents=agent)
    trainer.train()

    env.close()


if __name__ == "__main__":
    main()
