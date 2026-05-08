---
wave: 1
---
# Phase 13 Plan: Remote Training Telemetry (Dockerized TensorBoard)

This phase focuses on setting up a persistent, remote-accessible dashboard for viewing Stable Baselines 3 training runs, replicating the seamless remote experience of existing containers (like Bento PDF).

*Note: The "Live Policy Inference GUI" (Rerun.io) has been deferred. The current priority is strictly surfacing the historical and ongoing training metrics (Learning Rate, Loss, Episodic Rewards) over the network.*

## 1. Context & Architecture Decisions
- **Source Data**: Stable Baselines 3 is already automatically generating TensorBoard logs in `arcproLab/logs/ppo/`.
- **Infrastructure**: We will deploy an official `tensorflow/tensorboard` Docker container.
- **Networking**: The container will mount the local `logs/` directory and expose a port (e.g., `6006`) to the host machine, allowing the user to view the training graphs remotely via `http://<host-ip>:6006`.

## 2. Goals
1. **Docker Integration**: Create a robust method (e.g., `docker-compose.yml` or a launch script) to spin up the TensorBoard container, ensuring it automatically syncs with new data written by `train_policy.py`.
2. **Path Mapping**: Correctly map the host's `arc_rl_isacc_sim/logs` directory to the container's expected `--logdir`.

---

## 3. Implementation Plan

### Wave 1: Docker Configuration
- [ ] **Task 13-01-tensorboard-docker**: 
    - Create a `docker-compose.yml` (or a `launch_tensorboard.sh` script) at the project root.
    - Configure it to use the `tensorflow/tensorboard:latest` image.
    - Mount the `./logs` directory as a volume.
    - Bind host port `6006` to container port `6006`.
    - Set the entrypoint command: `tensorboard --logdir /logs --host 0.0.0.0 --reload_multifile=true`.

### Wave 2: Deployment & Verification
- [ ] **Task 13-02-remote-verification**:
    - Launch the container.
    - Verify that navigating to the exposed port in a remote browser successfully loads the historical PPO training data.
    - Confirm that starting a new `train_policy.py` run dynamically updates the remote TensorBoard UI without needing a container restart.