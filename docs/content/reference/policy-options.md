---
title: Policy options
---

# Policy options

This page documents fields read by the policy parser. See [the tutorial](../tutorial/05-policy-configuration.md) or the [fullExample](https://github.com/isri-aist/new-rl-qp-controller/tree/main/policies/fullExample/policy.yaml) of the template for a complete example.

`policy.yaml` lives alongside `observations.yaml` and an ONNX file in its own folder under `policies_root`.

## Minimal example

If the ONNX file carries training metadata (see [ONNX metadata](#onnx-metadata)), this is enough:

```yaml
action: {}
control: {}
```

Without metadata, `control.kp`/`control.kd` are required and `action.joints` needs a joint group from a convention; everything else falls back to a default:

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
| `use_onnx_metadata` | No | `true` | Read the training data stored in the ONNX file, see [ONNX metadata](#onnx-metadata). |

## `action`

| Field | Required | Default | Description |
|---|---:|---|---|
| `joints` | No | `all` | Name of a joint group, defining ONNX output order and size. The `all` group comes from the convention or from the ONNX metadata. Unlike observations' `joints` parameter (see [Available observations](observations.md#common-parameters)), this must be a named group. See [Training conventions §Is a convention required?](conventions.md#is-a-convention-required). |
| `controlled_joints` | No | `joints` | Applied subset; must be contained in `joints`. |
| `scale` | No | ONNX metadata, then `1.0` | Scalar or per-joint map. The ONNX value wins for the joints it covers. |
| `q0` | No | ONNX metadata, then posture task | Per-joint map. The ONNX value wins for the joints it covers; joints with no value use the posture task. |

## `control`

| Field | Required | Default | Description |
|---|---:|---|---|
| `use_QP` | No | `true` | QP or bypass. |
| `period_s` | No | `0.02` | Inference period in seconds. |
| `frequency_hz` | No | — | Alternative to `period_s`. |
| `pd_gains_ratio` | No | `1.0` | Gain scaling. |
| `phase_period` | No | observation value, then `1.0` | Phase period. |
| `kp` | Without metadata | ONNX metadata | Base stiffness map. The `policy.yaml` value wins. Every controller joint needs a value from one of the two. |
| `kd` | Without metadata | ONNX metadata | Base damping map. Same rules as `kp`. |

Every key of the `kp`, `kd`, `scale` and `q0` maps must be a controller joint: a misspelled name stops loading.

Specify only one of `period_s` and `frequency_hz`.

## ONNX metadata { #onnx-metadata }

Some exporters (mjlab, for example) store training data in the ONNX file as metadata. When present, it is read at startup for every policy and used as follows:

| Metadata key | Used for | If `policy.yaml` also sets it |
|---|---|---|
| `joint_names` | the `all` joint group: action and observation joint order | the convention's `all` group must be identical, otherwise loading fails |
| `default_joint_pos` | `action.q0` | **ONNX wins**, warning on differences |
| `action_scale` | `action.scale` | **ONNX wins**, warning on differences |
| `joint_stiffness` | `control.kp` | **`policy.yaml` wins**, warning on differences |
| `joint_damping` | `control.kd` | **`policy.yaml` wins**, warning on differences |
| `observation_names` | cross-check of the `observations.yaml` order | warning only |

To see what a file contains, run `python3 scripts/inspect_policy.py <model.onnx>`: it lists these keys, the joint order with its per-joint values, and the observation terms.

Differences smaller than rounding (1e-3 + 1% of the ONNX value) are ignored. Joints missing from the metadata (for example joints the policy does not control) still need `policy.yaml` values. 

You can set `use_onnx_metadata: false` to ignore the metadata entirely if needed.

## Validation failures

Loading fails for :

- unresolved paths/names
- an unknown joint group in `action.joints` (no silent fallback to the controller order), including `all` when neither the ONNX metadata nor the convention provides it
- an `action.joints` that is not a group name, such as an inline list
- a convention `all` group that differs from the ONNX `joint_names`
- a joint in the ONNX `joint_names` that the robot does not have
- a `kp`/`kd`/`scale`/`q0` key that is not a controller joint
- simultaneous period and frequency
- non-positive timing or gain ratio
- a controller joint with no `kp` or `kd`, in `policy.yaml` nor in the ONNX metadata
- invalid controlled subset
- ONNX output mismatch.
