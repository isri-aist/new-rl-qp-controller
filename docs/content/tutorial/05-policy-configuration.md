---
title: "5. Configure the policy"
---

# 5. Configure the policy

This step creates `policies/unitree/policy.yaml`. For the full field reference, see [Policy options](../reference/policy-options.md) — this page derives every value for the `unitree_rl_mjlab` G1 velocity policy and gives you the complete file.

## Create the folder

```bash
mkdir -p policies/unitree
```

The policy will be named after this folder (`unitree`), matching the `default_policy: unitree` already set in [Configure the controller](02-controller-configuration.md).

## Finding the information

We are going to check their velocity policy folder for G1 : [`deploy/robots/g1/config/policy/velocity/v0`](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/deploy/robots/g1/config/policy/velocity/v0) 

```text
deploy/robots/g1/config/policy/velocity/v0/
├── exported/
│   └── policy.onnx
└── params/
    └── deploy.yaml
```

**`params/deploy.yaml`** is what this step actually translates — it's the exact deployment spec the policy was exported with: per-joint stiffness/damping, default pose andaction scaling. It also has the observation list in training order, that we need for [Observation configuration](./06-observations-configuration.md).

```yaml title="deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml (excerpt)"
step_dt: 0.02
stiffness: [40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 28.5, 28.5,
            14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8]
damping:   [2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 1.8, 1.8,
            0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1]
default_joint_pos: [-0.1,0,0,0.3,-0.2,0, -0.1,0,0,0.3,-0.2,0,  0,0,0,  0.35,0.18,0,0.87,0,0,0, 0.35,-0.18,0,0.87,0,0,0]

commands:
  base_velocity:
    ranges:
      lin_vel_x: [-0.5, 1.0]
      lin_vel_y: [-0.5, 0.5]
      ang_vel_z: [-1.0, 1.0]

actions:
  JointPositionAction:
    scale: [0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44,
            0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07]
    offset: [-0.1,0,0,0.3,-0.2,0, -0.1,0,0,0.3,-0.2,0,  0,0,0,  0.35,0.18,0,0.87,0,0,0, 0.35,-0.18,0,0.87,0,0,0]
```

All arrays follow the same 29-joint order as `conventions.yaml`'s `all` group from the previous step (`joint_ids_map` in the same file is the identity `[0..28]`, confirming no reindexing is needed).

## Field-by-field mapping

| `deploy.yaml` | `policy.yaml` | Notes |
|---|---|---|
| `stiffness` | `control.kp` | Direct copy, per joint name. |
| `damping` | `control.kd` | Direct copy, per joint name. |
| `default_joint_pos` | `action.q0` | This is the pose `q_rl` uses at every policy update, adding to it the scaled action — see [Architecture: default pose](../reference/architecture.md#default-pose). |
| `actions.JointPositionAction.scale` | `action.scale` | Per-joint action scale. |
| `step_dt` | `control.period_s` | The policy's inference period. |
| `commands.base_velocity.ranges` | `control.vel_cmd_speed` | **Not a direct copy** — see below. |
| (observation `gait_phase.params.period`) | `control.phase_period` | `0.6`, from the `gait_phase` observation's params in the same file — used again in the next step. |

`action.joints: all` and `control.use_QP: true` come from the convention and from wanting the CBF-QP safety layer enabled, not from `deploy.yaml`.

### `vel_cmd_speed` is a judgment call

`commands.base_velocity.ranges` describes the *training* distribution: `lin_vel_x` is asymmetric (`-0.5` to `1.0`), `lin_vel_y` and `ang_vel_z` are symmetric. `control.vel_cmd_speed` in `policy.yaml`, by contrast, sets the *symmetric* maximum magnitude the GUI/joystick command widget will ever send (see [Available observations: `command`](../reference/observations.md#command)). There's no single correct way to collapse an asymmetric training range into a symmetric runtime limit — this integration picked:

- `xy: 0.5` — the lateral bound and the backward bound of the trained range, applied uniformly; conservative on the forward side (which trained up to `1.0`) in exchange for a simple, symmetric joystick mapping.
- `yaw: 1.0` — the trained range is already symmetric, so no compromise needed.

Revisit this if you want to command higher forward speeds and are comfortable operating closer to the edge of the trained distribution.

## The complete file

```yaml title="policies/unitree/policy.yaml"
# Full policy configuration reference.
# Every policy-level field currently accepted by the controller is represented.

# Optional. Defaults to the policy directory name.
name: unitree

# Optional. Defaults to <name>.onnx.
onnx: policy.onnx

action:
  # Optional. Defaults to all. Joint group defining the ONNX action vector order and size.
  joints: all

  # Optional. Defaults to action.joints. Used when only a subset of joints should be applied (ex: policy outputs all joints while only legs should be used)
  # controlled_joints: joint_subset

  # Optional. May be either one scalar for every joint...
  # scale: 1.0
  # ...or a complete or partial per-joint map. Unspecified joints use 1.0.
  scale:
    left_hip_pitch_joint: 0.55
    left_hip_roll_joint: 0.35
    left_hip_yaw_joint: 0.55
    left_knee_joint: 0.35
    left_ankle_pitch_joint: 0.44
    left_ankle_roll_joint: 0.44
    right_hip_pitch_joint: 0.55
    right_hip_roll_joint: 0.35
    right_hip_yaw_joint: 0.55
    right_knee_joint: 0.35
    right_ankle_pitch_joint: 0.44
    right_ankle_roll_joint: 0.44
    waist_yaw_joint: 0.55
    waist_roll_joint: 0.44
    waist_pitch_joint: 0.44
    left_shoulder_pitch_joint: 0.44
    left_shoulder_roll_joint: 0.44
    left_shoulder_yaw_joint: 0.44
    left_elbow_joint: 0.44
    left_wrist_roll_joint: 0.44
    left_wrist_pitch_joint: 0.07
    left_wrist_yaw_joint: 0.07
    right_shoulder_pitch_joint: 0.44
    right_shoulder_roll_joint: 0.44
    right_shoulder_yaw_joint: 0.44
    right_elbow_joint: 0.44
    right_wrist_roll_joint: 0.44
    right_wrist_pitch_joint: 0.07
    right_wrist_yaw_joint: 0.07

  # Optional. Joints with no value use the ONNX metadata, then the posture task.
  q0:
    left_hip_pitch_joint: -0.1
    left_hip_roll_joint: 0
    left_hip_yaw_joint: 0
    left_knee_joint: 0.3
    left_ankle_pitch_joint: -0.2
    left_ankle_roll_joint: 0
    right_hip_pitch_joint: -0.1
    right_hip_roll_joint: 0
    right_hip_yaw_joint: 0
    right_knee_joint: 0.3
    right_ankle_pitch_joint: -0.2
    right_ankle_roll_joint: 0
    waist_yaw_joint: 0
    waist_roll_joint: 0
    waist_pitch_joint: 0
    left_shoulder_pitch_joint: 0.35
    left_shoulder_roll_joint: 0.18
    left_shoulder_yaw_joint: 0
    left_elbow_joint: 0.87
    left_wrist_roll_joint: 0
    left_wrist_pitch_joint: 0
    left_wrist_yaw_joint: 0
    right_shoulder_pitch_joint: 0.35
    right_shoulder_roll_joint: -0.18
    right_shoulder_yaw_joint: 0
    right_elbow_joint: 0.87
    right_wrist_roll_joint: 0
    right_wrist_pitch_joint: 0
    right_wrist_yaw_joint: 0

control:
  # Optional. Defaults to true.
  use_QP: true

  # Optional policy period. Defaults to 0.02 s.
  # Specify either one of period_s and frequency_hz.
  period_s: 0.02
  # frequency_hz: 50.0

  # Optional. Defaults to 1.0.
  pd_gains_ratio: 1.0

  # Optional. Maximum symmetric velocity command magnitude exposed to the GUI/joystick.
  vel_cmd_speed:
    xy: 0.5
    yaw: 1.0

  # Optional unless a phase observation is used. Defaults to 1.0 s.
  phase_period: 0.6

  # Every joint in robot().refJointOrder() needs a value, here or in the ONNX metadata.
  kp:
    left_hip_pitch_joint: 40.2
    left_hip_roll_joint: 99.1
    left_hip_yaw_joint: 40.2
    left_knee_joint: 99.1
    left_ankle_pitch_joint: 28.5
    left_ankle_roll_joint: 28.5
    right_hip_pitch_joint: 40.2
    right_hip_roll_joint: 99.1
    right_hip_yaw_joint: 40.2
    right_knee_joint: 99.1
    right_ankle_pitch_joint: 28.5
    right_ankle_roll_joint: 28.5
    waist_yaw_joint: 40.2
    waist_roll_joint: 28.5
    waist_pitch_joint: 28.5
    left_shoulder_pitch_joint: 14.3
    left_shoulder_roll_joint: 14.3
    left_shoulder_yaw_joint: 14.3
    left_elbow_joint: 14.3
    left_wrist_roll_joint: 14.3
    left_wrist_pitch_joint: 16.8
    left_wrist_yaw_joint: 16.8
    right_shoulder_pitch_joint: 14.3
    right_shoulder_roll_joint: 14.3
    right_shoulder_yaw_joint: 14.3
    right_elbow_joint: 14.3
    right_wrist_roll_joint: 14.3
    right_wrist_pitch_joint: 16.8
    right_wrist_yaw_joint: 16.8
  kd:
    left_hip_pitch_joint: 2.6
    left_hip_roll_joint: 6.3
    left_hip_yaw_joint: 2.6
    left_knee_joint: 6.3
    left_ankle_pitch_joint: 1.8
    left_ankle_roll_joint: 1.8
    right_hip_pitch_joint: 2.6
    right_hip_roll_joint: 6.3
    right_hip_yaw_joint: 2.6
    right_knee_joint: 6.3
    right_ankle_pitch_joint: 1.8
    right_ankle_roll_joint: 1.8
    waist_yaw_joint: 2.6
    waist_roll_joint: 1.8
    waist_pitch_joint: 1.8
    left_shoulder_pitch_joint: 0.9
    left_shoulder_roll_joint: 0.9
    left_shoulder_yaw_joint: 0.9
    left_elbow_joint: 0.9
    left_wrist_roll_joint: 0.9
    left_wrist_pitch_joint: 1.1
    left_wrist_yaw_joint: 1.1
    right_shoulder_pitch_joint: 0.9
    right_shoulder_roll_joint: 0.9
    right_shoulder_yaw_joint: 0.9
    right_elbow_joint: 0.9
    right_wrist_roll_joint: 0.9
    right_wrist_pitch_joint: 1.1
    right_wrist_yaw_joint: 1.1
```

## Where you should be now

`policies/unitree/policy.yaml` exists and is complete. The folder still has no ONNX file and no `observations.yaml` — the controller won't load this policy yet. Next: the observations.
