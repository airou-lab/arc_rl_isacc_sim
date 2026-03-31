# USD Asset Archive

This directory contains legacy and experimental USD assets generated during the Phase 05-01 Asset Verification process.

## Asset Catalog

### Original OSM Assets
*   **`original_usd.usd`**
    *   **Source**: Raw export from the OpenStreet Map (OSM) tool.
    *   **Status**: Physically broken. Meshes are 2D (paper-thin), causing small car wheels to "tunnel" and fall into the void.
*   **`arcpro_RL_open_street_sim.usd`**
    *   **Role**: Original "Master" stage containing references to all tiles and metadata.

### Production & Troubleshooting Variants
*   **`original_production.usd`**
    *   **Source**: Created during the "Deep Dive" investigation (Bake & Harden script).
    *   **Status**: A "baked" version where all 527 meshes are local and have physical volume/friction.
    *   **Issue**: While physically "solid," it is **bumpy**. The robot hits micro-cliffs at every tile junction and gets stuck.
*   **`original_production_v2.usd`**
    *   **Status**: Alternative production variant.
*   **`original_thickened.usd`**, **`original_hardened.usd`**, **`original_flattened.usd`**
    *   **Role**: Interim "snapshots" of the troubleshooting process. These helped identify mesh thinness as the root cause of the car falling through.

### Legacy Isaac Sim Assets
*   **`no_graph_sim.usd`**, **`no_graph_sim_cleaned.usd`**
    *   **Role**: Earlier versions of the track before the "Final" stable version was established.

---
*Note: Use `no_graph_sim_final.usd` in the parent directory for stable training and verification.*
