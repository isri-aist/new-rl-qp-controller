---
title: "4. Define the training convention"
---

# 4. Define the training convention

This step edits `policies/conventions.yaml`.

## Why specify a convention ?

This is mostly **optional**, but it will provide much cleaner and clearer policy configurations afterwards. Basic examples include :

- Specifying a name for a joint group in order, instead of having to list them at each observation or policy configuration that needs them.
- Default parameters for observations if multiple policies trained by the same training environment will be tested with this controller : this simply avoids repetition.  
...

For the full semantics of this file, see the [Training conventions reference](../reference/conventions.md) — this page is the concrete fill-in for the `unitree_rl_mjlab` G1 policy.


!!! warning

    An `all` entry containing all joints in the order used by the policy is **necessary**. This is needed to handle the policy appropriately. The rest of the options are completely optional.

## Where the joint order comes from

The convention's `joint_groups.all` must list every actuated joint in **exactly the order the ONNX model was trained and exported with**. For a policy trained with `unitree_rl_mjlab`, that order is published alongside the exported model, in the policy's `deploy.yaml` (see [Configure the policy](05-policy-configuration.md) for more on that file) as `joint_ids_map: [0, 1, 2, ..., 28]` — an identity mapping, meaning the deployment order *is* the training order, with no reindexing needed. The 29 joint names themselves come from the G1's own URDF/robot description, in the same order used throughout `unitree_rl_mjlab`'s G1 task configuration.

## The convention

```yaml
conventions:
#   general:
#     observation_defaults:
      # Optional : if any general observation default could to be specify, specify them here. Example :
      # base_orientation:
      #   sensor: Accelerometer
      #   scale: 1.0

  mjlab: # example : mjlab, isaaclab, custom
    joint_groups:
      # The order here must exactly match the policy training/export order.
      # These joint groups can be used to resolve the joint order/joints used by the observations, as well as the actions outputed by the policy.
      all: [
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
        ]
    #   joint_subset: [
    #     "jointName1",
    #     "jointName2"
    #     ]

    # type_aliases:
    #   # Optional : aliases can be specified for the observations (to fit their training environment name or else). Example :
    #   # builtin_sensor: force_sensor # (alias: available_observation)

    # observation_defaults:
    #   # Optional : if any convention-specific observation default could to be specify, specify them here. Example :
    #   # joint_pos:
    #   #   joints: joint_subset

# Add as many conventions as needed.
```

The convention is named `mjlab`, after the training environment — a naming choice, not something the controller interprets specially.

## `type_aliases` - empty here

[Training conventions](../reference/conventions.md#type_aliases) lets you write `observations.yaml` using the training environment's own names (e.g. `joint_pos_rel`) and have them resolve to the controller's internal type (`joint_pos`). If you're setting up a convention you expect to reuse across many policies from the same training environment, defining `type_aliases` once could be worth it.

## Where you should be now

`policies/conventions.yaml` now defines a real `mjlab` convention with the correct joint order. `grep -rn "TODO(robot)" .` should no longer flag anything in this file. Next: the policy itself and its configurations.
