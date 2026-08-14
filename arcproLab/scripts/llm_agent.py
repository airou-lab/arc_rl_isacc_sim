"""
Local LLM Self-Healing Agent for RL Training
=============================================
Replaces the external API-dependent monitor subagent with a local
Qwen 3 Coder model served via Ollama. Reads training metrics,
constructs structured prompts with full reward tuning history,
calls Ollama, and applies validated reward weight changes.

Architecture: Time-shared GPU
  - Training pauses → Ollama loads → LLM analyzes → fix applied → Ollama unloads → training restarts

Usage:
    python llm_agent.py                          # Run analysis cycle
    python llm_agent.py --model qwen3-coder:3b   # Use smaller model
    python llm_agent.py --dry-run                # Show proposed changes without applying
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
import urllib.error
from datetime import datetime
from pathlib import Path
from typing import Any

# ──────────────────────────────────────────────────────────────────
# Paths (relative to project root)
# ──────────────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
ENV_CFG_PATH = PROJECT_ROOT / "arcproLab" / "arcpro_env_cfg.py"
REWARDS_PATH = PROJECT_ROOT / "arcproLab" / "mdp" / "rewards.py"
HISTORY_PATH = PROJECT_ROOT / ".planning" / "reward_tuning_history.md"
LOG_PATH = PROJECT_ROOT / "logs" / "skrl_phase1.log"
DIAGNOSIS_PATH = PROJECT_ROOT / "logs" / "LLM_AGENT_DIAGNOSIS.md"
CHECKPOINT_DIR = PROJECT_ROOT / "logs" / "ppo_skrl"
TRAINING_SCRIPT = PROJECT_ROOT / "start_tmux_training.sh"

# ──────────────────────────────────────────────────────────────────
# Ollama configuration
# ──────────────────────────────────────────────────────────────────
OLLAMA_API_URL = "http://localhost:11434"
DEFAULT_MODEL = "qwen3-coder:8b"

# ──────────────────────────────────────────────────────────────────
# Guardrails: Allowed reward weight ranges
# Prevents the LLM from setting absurd values
# ──────────────────────────────────────────────────────────────────
REWARD_GUARDRAILS = {
    "survival_bonus":       {"min": -5.0,   "max": 10.0},
    "termination_penalty":  {"min": 0.0,    "max": 100.0},
    "progress_reward":      {"min": 0.0,    "max": 200.0},
    "action_steer_penalty": {"min": -20.0,  "max": 0.0},
    "action_drive_reward":  {"min": -5.0,   "max": 5.0},
    "lateral_error":        {"min": -20.0,  "max": 0.0},
    "stationary":           {"min": 0.0,    "max": 200.0},
    "heading":              {"min": 0.0,    "max": 50.0},
    "smoothness":           {"min": 0.0,    "max": 20.0},
    "jerk":                 {"min": -20.0,  "max": 0.0},
    "boundary":             {"min": 0.0,    "max": 5.0},
}


# ──────────────────────────────────────────────────────────────────
# Utility functions
# ──────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    """Timestamped console output."""
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[LLM-Agent {ts}] {msg}")


def ollama_request(endpoint: str, payload: dict, timeout: int = 300) -> dict:
    """Make a request to the Ollama API."""
    url = f"{OLLAMA_API_URL}{endpoint}"
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            # Ollama streams responses line-by-line (NDJSON)
            full_response = ""
            for line in resp:
                chunk = json.loads(line.decode("utf-8"))
                if "message" in chunk and "content" in chunk["message"]:
                    full_response += chunk["message"]["content"]
                elif "response" in chunk:
                    full_response += chunk["response"]
                if chunk.get("done", False):
                    break
            return {"content": full_response, "done": True}
    except urllib.error.URLError as e:
        return {"error": str(e), "done": False}


def ensure_ollama_running() -> bool:
    """Start Ollama server if not already running. Returns True if ready."""
    try:
        urllib.request.urlopen(f"{OLLAMA_API_URL}/api/tags", timeout=5)
        log("Ollama server is already running.")
        return True
    except Exception:
        pass

    log("Starting Ollama server...")
    subprocess.Popen(
        ["ollama", "serve"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    # Wait for server to come up (max 30 seconds)
    for i in range(30):
        time.sleep(1)
        try:
            urllib.request.urlopen(f"{OLLAMA_API_URL}/api/tags", timeout=2)
            log("Ollama server is ready.")
            return True
        except Exception:
            pass

    log("ERROR: Ollama server failed to start within 30 seconds.")
    return False


def stop_ollama() -> None:
    """Stop the Ollama server to free VRAM."""
    log("Stopping Ollama server to free VRAM...")
    subprocess.run(["pkill", "-f", "ollama serve"], capture_output=True)
    time.sleep(2)


def load_model(model: str) -> bool:
    """Pre-load a model into Ollama. Returns True if successful."""
    log(f"Loading model {model} into VRAM...")
    result = ollama_request("/api/chat", {
        "model": model,
        "messages": [{"role": "user", "content": "Hello"}],
        "stream": False,
    }, timeout=120)
    if "error" in result:
        log(f"ERROR loading model: {result['error']}")
        return False
    log(f"Model {model} loaded successfully.")
    return True


# ──────────────────────────────────────────────────────────────────
# Metrics extraction
# ──────────────────────────────────────────────────────────────────

def get_latest_metrics() -> dict[str, Any]:
    """Parse the latest training metrics from the log file."""
    if not LOG_PATH.exists():
        return {"error": "Log file not found"}

    tail = subprocess.check_output(["tail", "-n", "100", str(LOG_PATH)]).decode("utf-8")

    timesteps = re.findall(r"Step (\d+)", tail)
    rewards = re.findall(r"Rew: ([-\d\.]+)", tail)
    ep_lens = re.findall(r"Len: (\d+)", tail)
    speeds = re.findall(r"Spd: ([-\d\.]+)", tail)
    wps_cum = re.findall(r"WPs_cum: ([-\d]+)", tail)
    wp_rew = re.findall(r"WPΔ_rew: ([-\d\.]+)", tail)

    return {
        "timesteps": int(timesteps[-1]) if timesteps else 0,
        "reward": float(rewards[-1]) if rewards else 0.0,
        "ep_len": int(ep_lens[-1]) if ep_lens else 0,
        "speed": float(speeds[-1]) if speeds else 0.0,
        "wps_cum": int(wps_cum[-1]) if wps_cum else 0,
        "wp_delta_rew": float(wp_rew[-1]) if wp_rew else 0.0,
    }


def get_current_weights() -> dict[str, float]:
    """Parse current reward weights from arcpro_env_cfg.py."""
    content = ENV_CFG_PATH.read_text()
    weights = {}
    for name in REWARD_GUARDRAILS:
        # Match patterns like: name = RewTerm(..., weight=X.X)
        # or: name = RewTerm(..., weight=-X.X)
        pattern = rf'{name}\s*=\s*RewTerm\([^)]*weight\s*=\s*([-\d\.]+)'
        match = re.search(pattern, content)
        if match:
            weights[name] = float(match.group(1))
    return weights


# ──────────────────────────────────────────────────────────────────
# Prompt construction
# ──────────────────────────────────────────────────────────────────

def build_prompt(metrics: dict, current_weights: dict) -> list[dict]:
    """Construct the structured prompt for the LLM."""

    # Read the full reward tuning history (truncate if too long for context)
    history = ""
    if HISTORY_PATH.exists():
        history = HISTORY_PATH.read_text()
        # Keep last 12000 chars to fit in context window (~3000 tokens)
        if len(history) > 12000:
            history = "...[TRUNCATED — showing last 12000 chars]...\n" + history[-12000:]

    # Read current rewards.py for code-level context
    rewards_code = ""
    if REWARDS_PATH.exists():
        rewards_code = REWARDS_PATH.read_text()

    system_prompt = """You are an expert RL reward engineer for an autonomous RC car training in NVIDIA Isaac Lab.

Your job is to diagnose WHY the agent is failing based on the metrics and reward tuning history, then propose WEIGHT CHANGES to fix it.

CRITICAL RULES:
1. You MUST read the FULL reward tuning history before proposing changes. DO NOT repeat past failed fixes.
2. You can ONLY change reward WEIGHTS in arcpro_env_cfg.py. You CANNOT modify reward function code.
3. Your response MUST be valid JSON (no markdown, no code fences, no explanation outside JSON).
4. Be conservative — change 1-3 weights at a time. Large sweeping changes destabilize training.
5. If the same exploit has persisted through 3+ fix attempts, suggest a STRUCTURAL change in the "notes" field.

RESPONSE FORMAT (strict JSON):
{
    "diagnosis": "Brief explanation of why the agent is failing",
    "issue_number": <next issue number>,
    "changes": [
        {"param": "<reward_name>", "old_value": <current>, "new_value": <proposed>, "reason": "why"}
    ],
    "wipe_checkpoints": true,
    "notes": "Any additional observations or structural suggestions"
}

AVAILABLE REWARD WEIGHTS (and what they do):
- survival_bonus: Points per step for being alive (currently causes spinning exploits when >0)
- termination_penalty: Multiplier for -50.0 crash penalty (weight=10 → -500 on crash)
- progress_reward: Multiplier for tanh(waypoint_progress) (main forward driving incentive)
- action_steer_penalty: Penalty for holding steering at extreme angles (negative weight = penalty)
- action_drive_reward: Reward for pressing gas (EXPLOITABLE — agent farms this while spinning)
- lateral_error: Penalty for drifting from centerline (Phase 2, usually disabled)
- stationary: Penalty for not advancing waypoints (fires every 10 steps if <5 WPs progress)
- heading: Reward for facing forward × speed (prevents backwards driving)
- smoothness: Penalty for jerky steering changes
- jerk: Penalty for steering oscillation (Phase 2, usually disabled)
- boundary: Multiplier for -100.0 boundary penalty (weight=0.6 → -60 near walls)
"""

    user_prompt = f"""## Current Training Metrics
```
Timesteps: {metrics.get('timesteps', 'N/A')}
Episode Reward: {metrics.get('reward', 'N/A')}
Episode Length: {metrics.get('ep_len', 'N/A')}
Speed (m/s): {metrics.get('speed', 'N/A')}
Cumulative Waypoints: {metrics.get('wps_cum', 'N/A')}
WP Delta Reward: {metrics.get('wp_delta_rew', 'N/A')}
```

## Current Reward Weights
```json
{json.dumps(current_weights, indent=2)}
```

## Reward Function Code (rewards.py)
```python
{rewards_code}
```

## Reward Tuning History (ALL previous issues and fixes)
{history}

Based on the metrics and history, diagnose the current failure mode and propose weight changes. Return ONLY valid JSON."""

    return [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt},
    ]


# ──────────────────────────────────────────────────────────────────
# Response parsing and validation
# ──────────────────────────────────────────────────────────────────

def parse_llm_response(raw: str) -> dict | None:
    """Parse and validate the LLM's JSON response."""
    # Strip markdown code fences if the LLM wraps them anyway
    cleaned = raw.strip()
    cleaned = re.sub(r"^```(?:json)?\s*", "", cleaned)
    cleaned = re.sub(r"\s*```$", "", cleaned)

    # Some models wrap in <think>...</think> tags — strip those
    cleaned = re.sub(r"<think>.*?</think>", "", cleaned, flags=re.DOTALL)
    cleaned = cleaned.strip()

    try:
        result = json.loads(cleaned)
    except json.JSONDecodeError as e:
        log(f"ERROR: Failed to parse LLM response as JSON: {e}")
        log(f"Raw response:\n{raw[:500]}")
        return None

    # Validate required fields
    required = ["diagnosis", "changes"]
    for field in required:
        if field not in result:
            log(f"ERROR: Missing required field '{field}' in LLM response")
            return None

    if not isinstance(result["changes"], list):
        log("ERROR: 'changes' must be a list")
        return None

    return result


def validate_changes(changes: list[dict], current_weights: dict) -> list[dict]:
    """Validate and clamp proposed changes against guardrails."""
    validated = []
    for change in changes:
        param = change.get("param", "")
        new_value = change.get("new_value")

        if param not in REWARD_GUARDRAILS:
            log(f"  REJECTED: '{param}' is not a recognized reward weight")
            continue

        if new_value is None:
            log(f"  REJECTED: '{param}' has no new_value")
            continue

        try:
            new_value = float(new_value)
        except (ValueError, TypeError):
            log(f"  REJECTED: '{param}' new_value '{new_value}' is not a number")
            continue

        # Clamp to guardrail range
        bounds = REWARD_GUARDRAILS[param]
        clamped = max(bounds["min"], min(bounds["max"], new_value))
        if clamped != new_value:
            log(f"  CLAMPED: '{param}' {new_value} → {clamped} (guardrail: [{bounds['min']}, {bounds['max']}])")
            new_value = clamped

        # Check it's actually different from current
        current = current_weights.get(param)
        if current is not None and abs(current - new_value) < 1e-6:
            log(f"  SKIPPED: '{param}' is already {current}")
            continue

        validated.append({
            "param": param,
            "old_value": current,
            "new_value": new_value,
            "reason": change.get("reason", "No reason given"),
        })

    return validated


# ──────────────────────────────────────────────────────────────────
# Apply changes to files
# ──────────────────────────────────────────────────────────────────

def apply_weight_changes(changes: list[dict]) -> bool:
    """Apply validated weight changes to arcpro_env_cfg.py."""
    content = ENV_CFG_PATH.read_text()
    modified = content

    for change in changes:
        param = change["param"]
        old_val = change["old_value"]
        new_val = change["new_value"]

        # Match the weight= value in the RewTerm for this param
        pattern = rf'({param}\s*=\s*RewTerm\([^)]*weight\s*=\s*)([-\d\.]+)'
        match = re.search(pattern, modified)
        if match:
            old_in_file = match.group(2)
            modified = modified[:match.start(2)] + str(new_val) + modified[match.end(2):]
            log(f"  APPLIED: {param} weight {old_in_file} → {new_val}")
        else:
            log(f"  WARNING: Could not find weight pattern for '{param}' in config file")

    if modified != content:
        ENV_CFG_PATH.write_text(modified)
        log(f"Saved changes to {ENV_CFG_PATH.name}")
        return True
    return False


def append_to_history(llm_result: dict, changes: list[dict]) -> None:
    """Append the new issue to reward_tuning_history.md."""
    issue_num = llm_result.get("issue_number", "??")
    diagnosis = llm_result.get("diagnosis", "No diagnosis")
    notes = llm_result.get("notes", "")
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M")

    entry = f"\n### Issue {issue_num}: [LLM-Agent Auto-Fix] ({timestamp})\n"
    entry += f"* **Problem**: {diagnosis}\n"
    entry += f"* **Fix**:\n"
    for c in changes:
        entry += f"  - `{c['param']}`: {c['old_value']} → {c['new_value']} ({c['reason']})\n"
    if notes:
        entry += f"* **Notes**: {notes}\n"
    entry += f"* **Result**: Applied by LLM agent. Checkpoints wiped, training restarted.\n"

    with open(HISTORY_PATH, "a") as f:
        f.write(entry)
    log(f"Appended Issue {issue_num} to reward_tuning_history.md")


def write_diagnosis(metrics: dict, llm_result: dict, changes: list[dict]) -> None:
    """Write a human-readable diagnosis file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    content = f"""# LLM Agent Diagnosis Report
**Generated**: {timestamp}
**Model**: Local Qwen 3 Coder via Ollama

## Metrics at Time of Analysis
| Metric | Value |
|--------|-------|
| Timesteps | {metrics.get('timesteps', 'N/A')} |
| Reward | {metrics.get('reward', 'N/A')} |
| Episode Length | {metrics.get('ep_len', 'N/A')} |
| Speed (m/s) | {metrics.get('speed', 'N/A')} |
| WPs Cumulative | {metrics.get('wps_cum', 'N/A')} |
| WP Delta Reward | {metrics.get('wp_delta_rew', 'N/A')} |

## Diagnosis
{llm_result.get('diagnosis', 'N/A')}

## Changes Applied
"""
    for c in changes:
        content += f"- **{c['param']}**: `{c['old_value']}` → `{c['new_value']}` — {c['reason']}\n"

    notes = llm_result.get("notes", "")
    if notes:
        content += f"\n## Additional Notes\n{notes}\n"

    DIAGNOSIS_PATH.write_text(content)
    log(f"Diagnosis written to {DIAGNOSIS_PATH.name}")


# ──────────────────────────────────────────────────────────────────
# Training control
# ──────────────────────────────────────────────────────────────────

def kill_training() -> None:
    """Kill the Isaac Lab training session."""
    log("Killing training session...")
    subprocess.run(["tmux", "kill-session", "-t", "training"], capture_output=True)
    subprocess.run(["pkill", "-f", "train_skrl.py"], capture_output=True)
    subprocess.run(["pkill", "-f", "train_policy.py"], capture_output=True)
    time.sleep(3)
    log("Training session killed.")


def wipe_checkpoints() -> None:
    """Delete all checkpoints to force fresh training."""
    if CHECKPOINT_DIR.exists():
        import shutil
        for item in CHECKPOINT_DIR.iterdir():
            if item.is_dir():
                shutil.rmtree(item)
            else:
                item.unlink()
        log("Checkpoints wiped.")
    else:
        log("No checkpoint directory found.")


def restart_training() -> None:
    """Restart the training via start_tmux_training.sh."""
    log("Restarting training...")
    subprocess.run(["bash", str(TRAINING_SCRIPT)], capture_output=True)
    time.sleep(5)

    # Verify tmux session exists
    result = subprocess.run(["tmux", "has-session", "-t", "training"], capture_output=True)
    if result.returncode == 0:
        log("Training restarted successfully in tmux session 'training'.")
    else:
        log("WARNING: tmux session 'training' not found after restart!")


# ──────────────────────────────────────────────────────────────────
# Main orchestration
# ──────────────────────────────────────────────────────────────────

def run_analysis_cycle(model: str = DEFAULT_MODEL, dry_run: bool = False) -> bool:
    """
    Execute one full analysis cycle:
      1. Read metrics
      2. Call LLM
      3. Validate + apply changes
      4. Restart training

    Returns True if changes were applied.
    """
    log("=" * 60)
    log("Starting LLM analysis cycle")
    log("=" * 60)

    # 1. Read current state
    metrics = get_latest_metrics()
    current_weights = get_current_weights()

    log(f"Current metrics: T={metrics.get('timesteps')} | Rew={metrics.get('reward')} | "
        f"Len={metrics.get('ep_len')} | Spd={metrics.get('speed')} | WPs={metrics.get('wps_cum')}")
    log(f"Current weights: {json.dumps(current_weights, indent=None)}")

    # 2. Start Ollama and load model
    if not ensure_ollama_running():
        log("FATAL: Cannot start Ollama. Aborting.")
        return False

    # 3. Construct prompt and call LLM
    messages = build_prompt(metrics, current_weights)
    log(f"Calling {model} via Ollama...")

    response = ollama_request("/api/chat", {
        "model": model,
        "messages": messages,
        "stream": True,
        "options": {
            "temperature": 0.3,  # Low temperature for structured output
            "num_predict": 1024,  # Cap output length
        },
    }, timeout=300)

    if "error" in response:
        log(f"LLM call failed: {response['error']}")
        stop_ollama()
        return False

    raw_content = response.get("content", "")
    log(f"LLM response received ({len(raw_content)} chars)")

    # 4. Free VRAM immediately after getting response
    stop_ollama()

    # 5. Parse and validate
    llm_result = parse_llm_response(raw_content)
    if llm_result is None:
        log("Failed to parse LLM response. Writing raw output to diagnosis file.")
        DIAGNOSIS_PATH.write_text(f"# LLM Agent — Parse Failure\n\nRaw response:\n```\n{raw_content}\n```\n")
        return False

    log(f"Diagnosis: {llm_result.get('diagnosis', 'N/A')}")

    changes = validate_changes(llm_result.get("changes", []), current_weights)
    if not changes:
        log("No valid changes to apply. Skipping.")
        write_diagnosis(metrics, llm_result, [])
        return False

    log(f"Validated {len(changes)} change(s):")
    for c in changes:
        log(f"  {c['param']}: {c['old_value']} → {c['new_value']}")

    # 6. Apply (or dry-run)
    if dry_run:
        log("DRY RUN — changes NOT applied.")
        write_diagnosis(metrics, llm_result, changes)
        return False

    # Kill training before modifying files
    kill_training()
    time.sleep(2)

    # Apply weight changes
    applied = apply_weight_changes(changes)
    if not applied:
        log("WARNING: No changes were written to the config file.")
        return False

    # Append to history
    append_to_history(llm_result, changes)

    # Write diagnosis
    write_diagnosis(metrics, llm_result, changes)

    # Wipe checkpoints if recommended
    if llm_result.get("wipe_checkpoints", True):
        wipe_checkpoints()

    # Restart training
    restart_training()

    log("=" * 60)
    log("Analysis cycle complete. Training restarted with new weights.")
    log("=" * 60)
    return True


# ──────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Local LLM Self-Healing Agent for RL Training")
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"Ollama model name (default: {DEFAULT_MODEL})")
    parser.add_argument("--dry-run", action="store_true", help="Show proposed changes without applying")
    args = parser.parse_args()

    success = run_analysis_cycle(model=args.model, dry_run=args.dry_run)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
