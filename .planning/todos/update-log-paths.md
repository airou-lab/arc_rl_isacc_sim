---
title: Update Log Paths
area: hygiene
status: pending
---

# Update Log Paths

Move all `.log` files into their own `logs/` folder in the project root (`/home/arika/Documents/arcpro/arcpro_system/src/examples/ARCPro_RL/arc_rl_isacc_sim/logs`).
The physical files have already been moved manually, but all scripts generating `.log` files must be updated so that all *new* logs are routed correctly to the `logs/` directory instead of the project root.
