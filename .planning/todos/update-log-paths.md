---
title: Update Log Paths
area: hygiene
status: pending
---

# Update Log Paths (COMPLETED)

Move all `.log` files into their own `logs/` folder in the project root (`/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs`).
The physical files have already been moved manually, and all scripts generating `.log` files have been updated to route all *new* logs correctly to the `logs/` directory.
