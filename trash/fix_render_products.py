
import os
from isaacsim import SimulationApp

# Initialize simulation app
simulation_app = SimulationApp({"headless": True})

import omni.usd
import omni.replicator.core as rep

# Load stage
usd_path = "/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/isacc_sim_usd/World0.usd"
omni.usd.get_context().open_stage(usd_path)
simulation_app.update()

print("\n--- Fixing Render Product Backgrounds ---")

# Use replicator to find all render products
render_products = rep.get.render_products()
for rp in render_products:
    print(f"Fixing Render Product: {rp}")
    # Set clear color to Black
    import omni.kit.commands
    omni.kit.commands.execute("ChangeProperty", prop_path=f"{rp}.inputs:clearColor", value=(0.0, 0.0, 0.0, 1.0), prev=None)

# Save stage
omni.usd.get_context().save_stage()
print("\nRender products fixed and stage saved.")

simulation_app.close()
