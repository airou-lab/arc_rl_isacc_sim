# HPPP Policy Contract (Protocol v2)

This document defines the strict interface requirements for the `HierarchicalPathPlanningPolicy` (HPPP). To use this policy, the environment MUST adhere to these observation and action spaces.

## 1. Observation Space (Dict)
The policy expects a Gymnasium `Dict` space with two keys:

### `image` (Box)
- **Shape:** `(90, 160, 3)`
- **Dtype:** `uint8`
- **Source:** Intel RealSense D435i (Downsampled)
- **Content:** RGB camera feed facing forward.

### `vec` (Box)
- **Shape:** `(12,)`
- **Dtype:** `float32`
- **Index Mapping:**
| Index | Constant | Name | Range | Description |
| :--- | :--- | :--- | :--- | :--- |
| 0 | `IDX_TURN_TOKEN` | Turn Token | `{-1, 0, 1}` | LEFT, STRAIGHT, RIGHT command. |
| 1 | `IDX_GO_SIGNAL` | Go Signal | `{0, 1}` | 1.0 = GO, 0.0 = WAIT/BRAKE. |
| 2 | `IDX_GOAL_DIST` | Goal Distance | `[0, inf)` | Masked to 0.0 for lane following. |
| 3 | `IDX_SPEED` | Forward Speed | `[0, inf)` | Metric scale (m/s). |
| 4 | `IDX_YAW_RATE` | Yaw Rate | `(-inf, inf)` | Angular velocity (rad/s). |
| 5 | `IDX_LAST_STEER` | Last Steer | `[-1, 1]` | Normalised steering action. |
| 6 | `IDX_LAST_THR` | Last Throttle | `[0, 1]` | Normalised throttle action. |
| 7 | `IDX_LAST_BRK` | Last Brake | `[0, 1]` | Normalised brake action. |
| 8 | `IDX_LAT_ERR` | Lateral Error | `(-inf, inf)` | Meters from lane center (Unmasked in reward). |
| 9 | `IDX_HDG_ERR` | Heading Error | `[-pi, pi]` | Radians from target heading. |
| 10 | `IDX_KAPPA` | Path Curvature | `(-inf, inf)` | 1/Radius of upcoming path. |
| 11 | `IDX_DS` | Distance Traveled | `[0, inf)` | Cumulative odometry (m). |

---

## 2. Action Space (Box)
- **Shape:** `(3,)`
- **Dtype:** `float32`
- **Mapping:**
  - `action[0]`: **Steering** (`[-1, 1]`) -> -1 is Left, +1 is Right.
  - `action[1]`: **Throttle** (`[0, 1]`) -> Forward drive.
  - `action[2]`: **Brake** (`[0, 1]`) -> Counter-velocity or deceleration.

### Integration Logic:
`Effective Velocity = Throttle * (1.0 - Brake) * MAX_SPEED`

---

## 3. Waypoint Normalization
- **WAYPOINT_NORM_SCALE:** `2.5`
- Predicted waypoints are in **Vehicle-Relative Frame** (X=Lateral, Y=Forward).
- The auxiliary loss back-projects actual trajectory into this frame for supervision.
