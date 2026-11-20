# Scripts

Executable entry points for training, evaluation, autonomous supervision, and CPU diagnostics:

- `train_skrl.py`: Main 48-env PPO training entry point (launched via `start_tmux_training.sh`).
- `play_skrl.py`: Deterministic evaluation and headed GUI playback (`debug.sh`).
- `autonomous_sentinel.py` & `sentinel_utils.py`: Live supervision daemon, VRAM/hang recovery, and JSONL event logging (`logs/sentinel_events.jsonl`).
- `checkpoint_probe.py` & `debias_checkpoint.py`: Fast pure-CPU equivariant steering-bias probes (`--equivariant --json`) and straightaway debiasing.
- `checkpoint_auditor.py`: 5-sector headless Isaac Sim regression gate.
- `audit_*.py`, `verify_*.py`, `test_*.py`: Targeted diagnostic and geometry verification utilities.
