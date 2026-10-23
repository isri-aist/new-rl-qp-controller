---
title: Policy options
---

# Policy options

This page documents fields read by the policy parser. See [the tutorial](../tutorial/05-policy-configuration.md) or the [fullExample](https://github.com/isri-aist/new-rl-qp-controller/tree/main/policies/fullExample/policy.yaml) of the template for a complete example.

`policy.yaml` lives alongside `observations.yaml` and an ONNX file in its own folder under `policies_root`.

## Minimal example

Only `action.joints` and `control.kp`/`control.kd` are strictly required; everything else falls back to a default:

```yaml
action:
  joints: all

control:
  kp:
    jointName1: 100.0
    jointName2: 100.0
    jointName3: 100.0
  kd:
    jointName1: 10.0
    jointName2: 10.0
    jointName3: 10.0
```

## Complete structure

```yaml
name: my_policy
onnx: my_policy.onnx

action:
  joints: all
  controlled_joints: legs
  scale: 0.25 # scalar or per-joint map
  q0:
    jointName1: 0.0
    jointName2: 0.0

control:
  use_QP: true
  period_s: 0.02
  # frequency_hz: 50.0
  pd_gains_ratio: 1.0
  phase_period: 1.0
  kp:
    jointName1: 100.0
    jointName2: 100.0
  kd:
    jointName1: 10.0
    jointName2: 10.0
```

## Top-level fields

| Field | Required | Default | Description |
|---|---:|---|---|
| `name` | No | folder name | Runtime name. |
| `onnx` | No | `<name>.onnx` | ONNX filename. |

## `action`

| Field | Required | Default | Description |
|---|---:|---|---|
| `joints` | Yes | — | Name of a `conventions.yaml` joint group, defining ONNX output order and size. Unlike observations' `joints` parameter (see [Available observations](observations.md#common-parameters)), this must be a named group — there's no inline-list alternative, so a convention is always required for this field. See [Training conventions §Is a convention required?](conventions.md#is-a-convention-required). |
| `controlled_joints` | No | `joints` | Applied subset; must be contained in `joints`. |
| `scale` | No | `1.0` | Action scale accepted by the runtime. |
| `q0` | No | controller/default posture | Full default-pose map when provided. |

## `control`

| Field | Required | Default | Description |
|---|---:|---|---|
| `use_QP` | No | `true` | QP or bypass. |
| `period_s` | No | `0.02` | Inference period in seconds. |
| `frequency_hz` | No | — | Alternative to `period_s`. |
| `pd_gains_ratio` | No | `1.0` | Gain scaling. |
| `phase_period` | No | observation value, then `1.0` | Phase period. |
| `kp` | Yes | — | Base stiffness map. |
| `kd` | Yes | — | Base damping map. |

Specify only one of `period_s` and `frequency_hz`.

## Validation failures

Loading fails for :

- unresolved paths/names
- missing or empty `action.joints`
- simultaneous period and frequency
- non-positive timing or gain ratio
- incomplete gain or provided `q0` maps
- invalid controlled subset
- ONNX output mismatch.
