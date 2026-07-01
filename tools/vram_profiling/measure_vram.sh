#!/bin/bash
# measure_vram.sh — Sidecar VRAM measurement. Runs nvidia-smi loggers ALONGSIDE
# a real training run. No edits to train_policy.py. Captures true device
# occupancy (CUDA context + PyTorch + Omniverse/Isaac renderer).
# Relocatable: finds repo root via `git rev-parse`. Outputs to <repo>/vram_runs/.
# Usage: ./measure_vram.sh <NUM_ENVS> [GPU_INDEX] [TIMESTEPS] [ISAACLAB_SH]
set -euo pipefail
NUM_ENVS="${1:?Usage: ./measure_vram.sh <NUM_ENVS> [GPU_INDEX] [TIMESTEPS] [ISAACLAB_SH]}"
GPU_INDEX="${2:-0}"; TIMESTEPS="${3:-100000}"
ISAACLAB_SH="${4:-/home/arika/IsaacLab/isaaclab.sh}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || echo "$SCRIPT_DIR")"
ENTRYPOINT="${REPO_ROOT}/arcproLab/scripts/train_policy.py"
ANALYZE="${SCRIPT_DIR}/vram_analyze.py"
if [[ ! -f "$ENTRYPOINT" ]]; then echo "ERROR: entrypoint not found at $ENTRYPOINT" >&2; exit 1; fi
if [[ ! -x "$ISAACLAB_SH" ]]; then echo "ERROR: isaaclab.sh not found/executable at $ISAACLAB_SH" >&2; exit 1; fi
TS="$(date +%Y%m%d-%H%M%S)"; OUTDIR="${REPO_ROOT}/vram_runs/${NUM_ENVS}env_${TS}"; mkdir -p "$OUTDIR"
GPU_CSV="${OUTDIR}/gpu.csv"; PROC_CSV="${OUTDIR}/procs.csv"; TRAIN_LOG="${OUTDIR}/train.log"; META="${OUTDIR}/meta.txt"
{ echo "num_envs=${NUM_ENVS}"; echo "gpu_index=${GPU_INDEX}"; echo "total_timesteps=${TIMESTEPS}";
  echo "entrypoint=${ENTRYPOINT}"; echo "isaaclab_sh=${ISAACLAB_SH}"; echo "repo_root=${REPO_ROOT}";
  echo "started=${TS}"; echo "host=$(hostname)"; } > "$META"
echo "=== VRAM measurement: num_envs=${NUM_ENVS}, gpu=${GPU_INDEX}, timesteps=${TIMESTEPS} ==="
echo "Output dir: ${OUTDIR}"
nvidia-smi -i "${GPU_INDEX}" --query-gpu=timestamp,index,memory.used,memory.total,utilization.gpu \
  --format=csv,nounits -lms 200 > "$GPU_CSV" & GPU_LOGGER_PID=$!
nvidia-smi -i "${GPU_INDEX}" --query-compute-apps=timestamp,pid,process_name,used_memory \
  --format=csv,nounits -lms 200 > "$PROC_CSV" & PROC_LOGGER_PID=$!
cleanup() { kill "$GPU_LOGGER_PID" "$PROC_LOGGER_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
sleep 2
echo "Launching training... (peak VRAM is in the OPTIMIZATION phase, not rollout)"
CUDA_VISIBLE_DEVICES="${GPU_INDEX}" "$ISAACLAB_SH" -p "$ENTRYPOINT" \
  --num_envs "${NUM_ENVS}" --headless --enable_cameras --total_timesteps "${TIMESTEPS}" \
  > "$TRAIN_LOG" 2>&1 & TRAIN_PID=$!
echo "train_pid=${TRAIN_PID}" >> "$META"; echo "Training PID: ${TRAIN_PID}  (tail -f ${TRAIN_LOG})"
wait "$TRAIN_PID"; TRAIN_RC=$?; echo "training_exit=${TRAIN_RC}" >> "$META"
sleep 1; cleanup; trap - EXIT INT TERM
echo ""; echo "=== Done. Training exit code: ${TRAIN_RC} ==="
if [[ -f "$ANALYZE" ]]; then python3 "$ANALYZE" peak "$GPU_CSV" "$PROC_CSV" || true; else echo "(vram_analyze.py not found next to this script)"; fi
echo "Artifacts in: ${OUTDIR}"
