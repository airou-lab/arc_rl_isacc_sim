import isaaclab.sim as sim_utils
try:
    cfg = sim_utils.SphereCfg(radius=0.2, collision_enabled=False)
    print("Success!")
except Exception as e:
    print(f"Error: {e}")
