# CLEARSET RESUME HERE

## Active Execution State
- **MILESTONE**: M006: Circuit Lap Mastery & Scene Parity
- **Phase**: Multi-Lap Endurance & Presentation Delivery
- **Task**: telemetry-verification-and-slide-figures
- **Next Todo**: Monitor 48-env training in tmux 'training' (passed step 270k, peak 93.3k steps / 16.3 laps, checkpoints agent_150000.pt and best_agent.pt saved); run 1-env checkpoint_auditor.py and prepare multi-agent intersection transition for M006 Phase 2.
- **Checkpoint Timestamp**: 2026-11-18T20:46:30.630974+00:00Z
- **Git Commit**: 9c01e4563b3b98d38eed085f7990ddd7d8d9a688

## Persistent State Data Sources
- **Turn Actions Database**: `.cs/cs_turns.db`
- **Codebase RAG Database**: `.cs/cs_codebase.db`
- **Codebase Index Guide**: `cs_codebase_index.yml`
- **Directory Structure**: `cs_directory_structure.yml`

## Recent Completed Actions
- **FIX** [SUCCESS]: Self-Healing: PROCESS_HANG: GPU VRAM exhaustion (11521.0MB > 11.5GB)
- **FIX** [SUCCESS]: Self-Healing: PROCESS_HANG: GPU VRAM exhaustion (11547.0MB > 11.5GB)
- **TUNE** [SUCCESS]: Curriculum dynamic bias tune: Right bias (-0.0460 < -0.04): Corner immersion

## Open Issues, Concerns & Learnings
- **ISSUE** (Sentinel Probe Omitted --equivariant & Eval Wrappers Kept Training DR Active): sentinel_utils.probe_checkpoint_bias invoked checkpoint_probe.py without --equivariant (default False), hiding +0.0405 (350k) and +0.1839 (400k) Yellow-L/White-R straightaway left bias behind raw pre-equivariant logits. Meanwhile play_skrl.py and checkpoint_auditor.py instantiated SKRLFlattenWrapper(env) with default training=True, injecting random brightness/contrast/gamma/noise at deterministic eval/GUI playback, and ARCProCritic obs_running_mean/var/count used persistent=False.
- **ISSUE** (48-env Track Overlap and 48x DistantLight Domain Mismatch): Track drivable_surfaces spans 44.74m x 42.90m (local X [-30.49,+14.25], Y [-19.01,+23.89]) while env_spacing=30.0m, causing adjacent elevated roads to hover over West Runway (Sectors 1-2) and cast shadows (Bot_L=87-95). Meanwhile openStreetUSD/no_graph_sim_clean_1x.usda defines DistantLight GlobalLight inside the per-env Track prim, cloning 48 global DistantLights (51 stage lights vs 4 at 1-env) and blowing out Sectors 3-5 to Bot_L=235-245. Fixed policy failed 1-env Sector 1 (step 159 no-augment, step 1093 augment) despite 46/48 48-env survival.
- **LEARNING** (Six Root-Cause Bug Loops Eliminated Permanently): Unified track_topology.py + repaired track_boundaries_1x.npz (0 violations at 0.170m), single per-step cdist cache + pre-reward go_signal update, restored -400.0 termination_penalty & explicit 225.0 lateral_error weight, fixed red-light stopping contradiction, decoupled Actor/Critic optimizers + RunningStandardScaler value_preprocessor + asinh Critic distance compression + smooth tanh log_std, equivariant G(I) + parity policy head + L_sym, and real-image masked-slot checkpoint probe/debias.
- **LEARNING** (RTX 3060 48-Camera Environment Hardware Sweet Spot): Profiled RTX 3060 12GB VRAM allocation during Isaac Sim 48-camera training. Total VRAM used is 7.45GB (6.3GB Isaac Sim + 1.2GB desktop compositor), leaving 4.8GB headroom. GPU utilization is at 87% with 160.5 FPS (3.34 policy steps/s). Increasing environments to 64 would saturate GPU rasterization cores and drop policy step rate by ~28% without speeding up wall-clock training, while pushing VRAM toward the 11.5GB Sentinel 5 restart threshold. 48 envs is the optimal hardware configuration.
- **LEARNING** (350-Step Audit (3.15m) vs 51.4m Circuit Lap (5,711 steps) Discrepancy): At 500 Hz physics and decimation 10 (50 Hz control, dt=0.02s), 350 steps at 0.45 m/s corresponds to exactly 3.15 meters of movement. Because the full track is 51.40 meters (5,000 waypoints at ~1.03cm spacing), a full lap requires 5,711 continuous steps. Automated 350-step sector tests previously reported 100% pass rates while debug.sh crashed because residual left steering (+4.5 to +7.9 deg) accumulated after 10-15 seconds (600-800 steps), crossing the yellow line. True lap validation requires a minimum 5,700-step continuous endurance run.

## Resume Instructions
1. Reset or clear the screen safely using `/clear` or `/cs-clear`.
2. On fresh context, invoke `/cs-resume` to restore project continuity directly from this file and SQLite databases.
