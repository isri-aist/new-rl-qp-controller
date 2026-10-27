---
title: Training conventions
---

# Training conventions

A convention, specified in `conventions.yaml`, describes how controller-side data is translated into the representation expected by a training environment.

It can define:

- the policy-facing joint order;
- named joint subsets used by observations and actions;
- aliases between training-environment observation names and registered controller observation types.

Set the convention in `observations.yaml`:

```yaml
training_convention: mc_rtc
```

A convention changes representation, not semantics. Always compare it with the actual training configuration rather than relying only on the convention name.

## Is a convention required? { #is-a-convention-required }

A convention is **optional**: it is a help to keep policy configurations short and clear.

The one thing the controller needs is the policy's joint order, as the `all` joint group. `action.joints` uses it by default. It comes from, in order:

1. the ONNX metadata `joint_names`, when the exporter stored it (see [ONNX metadata](policy-options.md#onnx-metadata)), in which case `training_convention` can be omitted entirely;
2. otherwise, the `all` group of the policy's convention.

!!! tip "Check the ONNX file first"
    Before writing a convention, run `python3 scripts/inspect_policy.py <model.onnx>` (it needs `pip install onnxruntime`). Its `Metadata` section tells all the fields provided, including `joint_names`, and prints them.

!!! warning
    `all` is needed. It will fail if the ONNX metadata do not provide it and neither does the `conventions.yaml` fie. If both provide it, they must be identical.

Named groups exist to avoid repeating the same joint list across many observations, and across every policy that shares a convention — reach for one once a list like `legs` above shows up more than a couple of times; inline it directly when it doesn't. `type_aliases` and `observation_defaults` are optional in the same spirit: conveniences for policies that would otherwise repeat themselves, not requirements.

## Location

Conventions are shared by every policy and are defined in `<policies_root>/conventions.yaml`:

```text
policies/
├── conventions.yaml
└── my_policy/
    ├── my_policy.onnx
    ├── policy.yaml
    └── observations.yaml
```

It can be useful to check it before writing a new policy's `policy.yaml` and `observations.yaml`.

## Minimal example

```yaml
conventions:
  my_convention:
    joint_groups:
      all: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll, ...]
      # other example : legs: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
```

When the ONNX file has no `joint_names` metadata, this is the minimum: one convention with an `all` group whose order matches your training exactly.

Joint groups can be used to specify either the action joints or the joints to consider for a specific observation.

!!! warning "Defaults can produce unexpected behavior if overlooked"
    Convention defaults are easy to forget when deploying a new policy against an existing convention.
    Always check `observation_defaults` — both in `conventions.<name>` and in `conventions.general` — before assuming an observation uses only the parameters written in `observations.yaml`.

## File structure

| Path | Required | Purpose |
|---|---:|---|
| `conventions` | Yes | Root map containing named convention profiles. |
| `conventions.<name>.joint_groups` | Yes | Joint groups used by action mapping and joint-based observations. |
| `conventions.<name>.type_aliases` | No | Maps training-environment observation names to internal observation types. |
| `conventions.<name>.observation_defaults` | No | Convention-specific default observation parameters. Applied before `observations.yaml`, which can override them. |
| `conventions.general.observation_defaults` | No | Defaults shared by all conventions, applied before convention-specific ones. |

## Complete example

```yaml
conventions:
  general:
    observation_defaults:
      base_orientation:
        sensor: Accelerometer
        scale: 1.0
        ...
  
  mjlab_exemple:
    joint_groups:
      all: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
      legs: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
      arms: [shoulder, elbow]
      ...
    
    type_aliases:
      joint_pos_rel: joint_pos
      joint_vel_rel: joint_vel
      generated_commands: command
      ...
    
    observation_defaults:
      joint_pos:
        joints: all
        relative_to_default_pose: true
      joint_vel:
        joints: all
        relative_to_default_velocity: true
      ...
  ...
```

## joint_groups

Named joint groups define subsets and their order. They are used by:

- `action.joints` and `action.controlled_joints` in `policy.yaml`
- `parameters.joints` in observation entries in `observations.yaml`

**The joint order must match your training environment exactly.** Reordering or omitting joints will produce incorrect behavior.

Without ONNX `joint_names` metadata, define `all` to represent the complete actuated joint set in training order. With it, `all` comes from the ONNX file and does not need to be defined here.

```yaml
conventions:
  my_convention:
    joint_groups:
      all: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
      legs: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
      upper_body: [shoulder_r, elbow_r, shoulder_l, elbow_l]
```

## type_aliases

Training environments often name observations differently from the controller's internal types. `type_aliases` lets you use training names directly in `observations.yaml` without code changes.

For example, if your training environment calls sensor observation `builtin_sensor`:

```yaml
type_aliases:
  builtin_sensor: force_sensor
```

Then in `observations.yaml`, you can write `type: builtin_sensor` exactly as in training, and it will resolve to the internal `force_sensor` observation.

This keeps policy files readable and prevents transcription errors when migrating between environments.

See [Available observations](observations.md) for details on each type.

## observation_defaults

Convention defaults let you specify parameters that apply to most or all policies using that convention, avoiding repetition in every `observations.yaml`.

### General defaults

Defaults under `conventions.general.observation_defaults` apply to all policies using any convention:

```yaml
conventions:
  general:
    observation_defaults:
      projected_gravity:
        scale: 1.0
      base_orientation:
        sensor: Accelerometer
```

### Convention-specific defaults

Defaults under `conventions.<name>.observation_defaults` apply only to policies using that convention:

```yaml
conventions:
  mjlab:
    observation_defaults:
      joint_pos:
        joints: all
        relative_to_default_pose: true
        scale: 1.0
      joint_vel:
        joints: all
        relative_to_default_velocity: true
```

### Override order

1. General defaults are merged first
2. Convention-specific defaults are merged second
3. Local parameters in `observations.yaml` override all of the above

This means a policy's explicit parameters always take precedence.

## Validation checklist

- [ ] Every `joint_groups` entry order matches training/export order exactly.
- [ ] The `training_convention` referenced by each policy exists in this file (or is omitted when the ONNX metadata provides `joint_names`).
- [ ] All `type_aliases` values are registered internal observation types (see [Available observations](observations.md)).
- [ ] Every observation name used in `observations.yaml` resolves either directly or through an alias.
- [ ] `observation_defaults` — both in `conventions.general` and the convention profile — match your training setup.
- [ ] Convention-specific defaults can be overridden in policy-local `observations.yaml` parameters when needed.
