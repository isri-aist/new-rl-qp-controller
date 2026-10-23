---
title: Logging and GUI
---

# Logging and GUI

Logger prefixes depend on the repository:

```text
NewRLQPController_...
G1RLQPController_...
H1RLQPController_...
HRP5pRLQPController_...
```

## Common logger suffixes

| Suffix | Content |
|---|---|
| `_kp_base`, `_kd_base` | Unscaled policy gains |
| `_kp_current`, `_kd_current` | Applied gains |
| `_pd_gains_ratio` | Runtime gain ratio |
| `_RL_q` | Full controller-order target |
| `_RL_qZero` | Default pose |
| `_RL_currentObservation` | ONNX input |
| `_RL_currentAction` | Raw ONNX output |
| `_RL_currentActionScaled` | Scaled controller-order action |
| `_RL_actionScale` | Action scale |
| `_RL_command` | `[vx, vy, yaw_rate]` |
| `_currentPolicy` | Active policy |
| `_observationConvention` | Active convention |
| `_RL_phase` | Normalized phase |
| `_RL_policy_period_s` | Inference period |
| `_RL_update_count` | Policy update count |
| `_useQP` | QP state |

## Per-observation history

```text
<ControllerPrefix>_Observations_<observation-name>
```

History entries use `_t`, `_t-1`, and so on. Give repeated terms unique names.

## GUI panels

- **Policy:** policy/folder/convention/source/base body, dimensions, selection, reload.
- **PD Gains:** gain ratio and gain vectors.
- **Control:** QP toggle and limit-warning toggle.
- **Runtime:** controller/policy rates, update count, phase.
- **Command:** `vx`, `vy`, and `yaw_rate`.
