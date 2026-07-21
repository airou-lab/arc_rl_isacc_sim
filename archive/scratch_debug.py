import sys
sys.path.insert(0, "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim")

try:
    with open("arcproLab/scripts/play_skrl.py") as f:
        code = f.read()
    exec(code)
except Exception as e:
    import traceback
    traceback.print_exc()
