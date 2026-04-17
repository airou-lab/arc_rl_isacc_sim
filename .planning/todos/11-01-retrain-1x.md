# Todo: 11-01-retrain-1x
Goal: Train fresh PPO policy at 1.0x scale with 32 parallel environments.

## Tasks
- [x] Launch retraining with `DISPLAY=:0 bash train.sh --num_envs 32 --headless`.
- [/] Monitor reward convergence (Target: 1,000,000 timesteps). [IN PROGRESS]
- [ ] Verify the trained model (road_following_model.pth) in the GUI using `run_gui_verify.sh`.

## Notes
- **Auto-Centering**: TrackManager now procedurally centers waypoints between yellow and white markers.
- **Termination**: Strict boundaries at +/- 0.2m (LatErr) for 1x scale stability.
