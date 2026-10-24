import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play SKRL ARCPro Policy")
parser.add_argument(
    "--checkpoint",
    type=str,
    default=None,
    help="Path to SKRL checkpoint .pt file. Auto-detects latest best_agent.pt if omitted.",
)
parser.add_argument(
    "--num_envs", type=int, default=1, help="Number of parallel environments"
)
parser.add_argument(
    "--debug",
    action="store_true",
    default=False,
    help="Enable visual debug spheres in Isaac Sim (boundaries, gates, centerline, target lane)",
)
parser.add_argument(
    "--sector",
    type=int,
    default=None,
    choices=[1, 2, 3, 4, 5],
    help="Force specific spawn sector (1=Start Line, 2=C1 Runway, 3=Leg 2 Exit, 4=Intersection, 5=North Straight)",
)
parser.add_argument(
    "--camera_mode",
    type=str,
    choices=["chase", "top_down", "static"],
    default="chase",
    help="Camera tracking mode: 'chase' (3rd person behind car), 'top_down' (overhead tracking car & spheres), or 'static'",
)
parser.add_argument(
    "--steer_bias_trim",
    type=float,
    default=0.0,
    help="Steering bias trim applied during evaluation (default: 0.0)",
)
parser.add_argument(
    "--max_steps",
    type=int,
    default=None,
    help="Maximum number of evaluation steps to run before exiting (default: infinite loop)",
)
parser.add_argument(
    "--expert",
    action="store_true",
    default=False,
    help="Run hard-coded closed-loop Stanley expert controller to navigate through intersection and curves",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
ARCPRO_LAB_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)
if ARCPRO_LAB_DIR not in sys.path:
    sys.path.insert(0, ARCPRO_LAB_DIR)

# Auto-detect latest checkpoint across logs and archive directories
search_dirs = [
    os.path.join(ROOT_DIR, "logs"),
    os.path.join(ROOT_DIR, "archive", "landmark_checkpoints"),
]

if args_cli.checkpoint is None or args_cli.checkpoint.strip().lower() in (
    "latest",
    "default",
):
    candidates = []
    for s_dir in search_dirs:
        if os.path.exists(s_dir):
            for root, dirs, files in os.walk(s_dir):
                for f in files:
                    if f.endswith(".pt") and "corrupted" not in root:
                        candidates.append(os.path.join(root, f))
    if candidates:
        args_cli.checkpoint = max(candidates, key=os.path.getmtime)
        print(f"[debug.sh] Auto-detected latest checkpoint: {args_cli.checkpoint}")
    else:
        print("[debug.sh] WARNING: No checkpoint found. Running with random weights.")
elif not os.path.exists(args_cli.checkpoint):
    # Support shortcuts like 'agent_50000.pt' or '50000'
    query = args_cli.checkpoint.strip()
    target = os.path.basename(query)

    matches = []
    for s_dir in search_dirs:
        if os.path.exists(s_dir):
            for root, dirs, files in os.walk(s_dir):
                for f in files:
                    if f.endswith(".pt"):
                        clean_query = query.replace(".pt", "")
                        if f == target or clean_query in f or target in f:
                            matches.append(os.path.join(root, f))
    if matches:
        exact_matches = [
            m
            for m in matches
            if os.path.basename(m)
            in (f"agent_{clean_query}.pt", f"{clean_query}.pt", target)
        ]
        if exact_matches:
            matches = exact_matches
        args_cli.checkpoint = max(matches, key=os.path.getmtime)
        print(f"[debug.sh] Resolved checkpoint shortcut to: {args_cli.checkpoint}")
    else:
        print(
            f"[debug.sh] WARNING: Could not resolve checkpoint '{query}'. Checked: {search_dirs}"
        )

print("Importing torch...")
sys.stdout.flush()
import torch

print("Importing IsaacLab...")
sys.stdout.flush()
from isaaclab.envs import ManagerBasedRLEnv

print("Importing SKRL...")
sys.stdout.flush()
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG

print("Importing ARCPro modules...")
sys.stdout.flush()
import traceback

from agents.skrl_wrappers import SKRLFlattenWrapper, WarmupActionWrapper
from skrl.envs.wrappers.torch import IsaacLabWrapper

from arcproLab.agents.skrl_models import ARCProActor, ARCProCritic
from arcproLab.arcpro_env_cfg import ARCProEnvCfg

try:
    print("Configuring environment...")
    sys.stdout.flush()
    if args_cli.sector is not None:
        sector_names = {
            1: "Start Line",
            2: "C1 Runway",
            3: "Leg 2 Exit",
            4: "Intersection",
            5: "North Straight",
        }
        sector_thresholds = {
            1: (1.0, 1.0, 1.0, 1.0),
            2: (0.0, 1.0, 1.0, 1.0),
            3: (0.0, 0.0, 1.0, 1.0),
            4: (0.0, 0.0, 0.0, 1.0),
            5: (0.0, 0.0, 0.0, 0.0),
        }
        th = sector_thresholds[args_cli.sector]
        sector_weights = [0.0, 0.0, 0.0, 0.0, 0.0]
        sector_weights[args_cli.sector - 1] = 1.0
        os.environ["ARCPRO_FORCE_SECTOR"] = str(args_cli.sector)
        try:
            import mdp.curriculum_config as curr_cfg
            import mdp.events as mdp_events

            curr_cfg._CACHED_CONFIG["sector_weights"] = sector_weights
            curr_cfg._CACHED_THRESHOLDS = list(th)
            curr_cfg.get_sector_thresholds = lambda: th
            mdp_events.get_sector_thresholds = lambda: th
            print(
                f"[debug.sh] Forced deterministic spawn in Sector {args_cli.sector} ({sector_names.get(args_cli.sector, 'Unknown')})"
            )
        except Exception as e:
            print(f"[debug.sh] Failed to set sector override: {e}")

    env_cfg = ARCProEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.enable_cameras = True
    env_cfg.__post_init__()

    print("Instantiating ManagerBasedRLEnv...")
    sys.stdout.flush()
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode="human")
    env = WarmupActionWrapper(env, warmup_steps=10)

    print("Wrapping environment...")
    sys.stdout.flush()
    env = IsaacLabWrapper(env)
    env = SKRLFlattenWrapper(env)

    if args_cli.debug:
        print(
            "[debug.sh] Activating visual debug spheres (Yellow/White boundaries, Green gates, Cyan path, Magenta lane)..."
        )
        from mdp.track_manager import get_track_manager

        tm = get_track_manager(device="cuda:0")
        tm.debug = True
        tm.refresh_visuals()

    print("Setting up models...")
    sys.stdout.flush()
    state_space = getattr(env, "state_space", env.observation_space)
    models = {}
    models["policy"] = ARCProActor(env.observation_space, env.action_space, "cuda:0")
    models["value"] = ARCProCritic(state_space, env.action_space, "cuda:0")

    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    agent_cfg["experiment"]["write_interval"] = 0
    agent_cfg["experiment"]["checkpoint_interval"] = 0

    agent = PPO(
        models=models,
        memory=None,
        cfg=agent_cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device="cuda:0",
    )

    print(f"Loading checkpoint: {args_cli.checkpoint}")
    if args_cli.checkpoint is not None and os.path.exists(args_cli.checkpoint):
        try:
            agent.load(args_cli.checkpoint)
            print("Checkpoint loaded successfully via agent.load.")
        except Exception as e:
            print(
                f"Note: Full agent.load encountered ({e}). Loading policy network weights directly for evaluation..."
            )
            ckpt_data = torch.load(args_cli.checkpoint, map_location=agent.device)
            if isinstance(ckpt_data, dict):
                if "policy" in ckpt_data:
                    agent.policy.load_state_dict(ckpt_data["policy"])
                    print("Policy network weights loaded successfully.")
                if "value" in ckpt_data:
                    try:
                        agent.value.load_state_dict(ckpt_data["value"])
                        print("Value network weights loaded successfully.")
                    except Exception:
                        pass
            else:
                agent.policy.load_state_dict(ckpt_data)
                print("Raw policy weights loaded successfully.")
    elif args_cli.checkpoint is not None:
        print(f"ERROR: Checkpoint not found: {args_cli.checkpoint}")
        print("Proceeding with random weights for visual debug.")
    else:
        print("No checkpoint. Proceeding with random weights for visual debug.")

    agent.set_mode("eval")

    def update_tracking_camera(env, mode="chase"):
        if mode == "static":
            return
        try:
            base_env = env.unwrapped
            robot = base_env.scene["robot"]
            pos = robot.data.root_pos_w[0].cpu().numpy()
            quat = robot.data.root_quat_w[0].cpu().numpy()  # [w, x, y, z]

            if mode == "top_down":
                eye = [float(pos[0]), float(pos[1]), float(pos[2]) + 4.0]
                target = [float(pos[0]), float(pos[1]), float(pos[2])]
            else:  # chase
                w, x, y, z = (
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                )
                # Vehicle forward heading in XY world coordinates
                fx = 1.0 - 2.0 * (y**2 + z**2)
                fy = 2.0 * (x * y + w * z)
                norm = (fx**2 + fy**2) ** 0.5
                if norm > 1e-3:
                    fx /= norm
                    fy /= norm
                else:
                    fx, fy = 1.0, 0.0

                # 2.0m behind, 1.0m above, aiming 1.2m ahead through windshield
                eye = [
                    float(pos[0]) - 2.0 * fx,
                    float(pos[1]) - 2.0 * fy,
                    float(pos[2]) + 1.0,
                ]
                target = [
                    float(pos[0]) + 1.2 * fx,
                    float(pos[1]) + 1.2 * fy,
                    float(pos[2]) + 0.3,
                ]

            base_env.sim.set_camera_view(eye=eye, target=target)
        except Exception:
            pass

    print("Resetting environment...")
    obs, info = env.reset()
    if args_cli.debug:
        from mdp.track_manager import get_track_manager

        tm = get_track_manager(device="cuda:0")
        tm.debug = True
        tm.refresh_visuals()
    update_tracking_camera(env, mode=args_cli.camera_mode)

    print(f"Entering simulation loop (camera_mode='{args_cli.camera_mode}')...")
    step_count = 0
    total_steps = 0
    while simulation_app.is_running():
        with torch.no_grad():
            if args_cli.expert:
                base_unwrapped = env.unwrapped
                lat_err = float(
                    base_unwrapped.extras.get("lat_err", torch.tensor([0.0]))[0].item()
                )
                head_err = float(
                    base_unwrapped.extras.get("head_err", torch.tensor([0.0]))[0].item()
                )
                # Closed-loop Stanley expert controller (matches tests/test_manual_left_turn.py)
                target_steer = -2.5 * lat_err - 1.2 * head_err
                steer_val = float(
                    torch.clamp(torch.tensor(target_steer), min=-1.0, max=1.0).item()
                )
                drive_val = -0.40  # ~0.40 m/s cornering cruising speed
                action = torch.tensor([[steer_val, drive_val]], device=env.device)
            else:
                # Deterministic policy execution: Use mean action without stochastic exploration noise
                _, _, outputs = agent.act(obs, timestep=0, timesteps=0)
                action = outputs.get("mean_actions", None)
                if action is None:
                    action = agent.policy.compute({"states": obs})[0]

                # Full steering authority for cornering & lane following (matches training action space)
                action = action.clone()
                if args_cli.steer_bias_trim != 0.0:
                    action[:, 0] += args_cli.steer_bias_trim

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1
        total_steps += 1
        update_tracking_camera(env, mode=args_cli.camera_mode)

        steer = float(action[0, 0].item())
        throttle = float(action[0, 1].item())
        speed = (
            float(info["speed_mps"].float().mean().item())
            if "speed_mps" in info
            else 0.0
        )
        rew_val = float(reward.float().mean().item())
        wps = (
            float(info["track_wp_delta"].float().mean().item())
            if "track_wp_delta" in info
            else 0.0
        )
        done = bool((terminated | truncated)[0].item())

        # Real-time clearance metrics to boundary spheres (in centimeters)
        dist_w_cm = (
            float(info["dist_white"][0].item()) * 100.0 if "dist_white" in info else 0.0
        )
        dist_y_cm = (
            float(info["dist_yellow"][0].item()) * 100.0
            if "dist_yellow" in info
            else 0.0
        )
        lat_err_cm = (
            float(info["lat_err"][0].item()) * 100.0 if "lat_err" in info else 0.0
        )

        if step_count % 10 == 0 or done:
            status = " [BOUNDARY COLLISION / RESET]" if done else ""
            print(
                f"Step {step_count:>5} | Steer: {steer:>5.2f} | Spd: {speed:>4.2f}m/s | "
                f"White: {dist_w_cm:>5.1f}cm | Yellow: {dist_y_cm:>5.1f}cm | LatErr: {lat_err_cm:>+5.1f}cm | "
                f"WPs: {wps:>4.0f} | Rew: {rew_val:>6.1f}{status}"
            )
            sys.stdout.flush()

        if done:
            print(f"[RESET] Episode finished at step {step_count} — resetting.")
            step_count = 0
            update_tracking_camera(env, mode=args_cli.camera_mode)

        if args_cli.max_steps is not None and total_steps >= args_cli.max_steps:
            print(
                f"[play_skrl.py] Reached requested max_steps={args_cli.max_steps}. Exiting cleanly."
            )
            break

    print("Simulation loop ended.")
    env.close()

except Exception:
    print("=" * 50)
    print("ERROR CAUGHT IN PLAY_SKRL.PY:")
    traceback.print_exc()
    print("=" * 50)
    sys.stdout.flush()

simulation_app.close()
print("Clean shutdown complete.")
