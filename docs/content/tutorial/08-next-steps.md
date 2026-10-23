---
title: "8. Next steps"
---

# 8. Next steps

## What was built

Starting from the template's `main` branch, this tutorial:

1. Renamed the template's integration points to `G1RLQPController`.
2. Configured the controller for G1: base body, and an observer pipeline (`BodySensor`, `Tilt`, `ExternalForcesObserver`) matched to what G1 exposes.
3. Wired up the C++-side `anchorFrameFunction` the `Tilt` observer needs.
4. Defined the `mjlab` training convention with G1's 29-joint order.
5. Translated `unitree_rl_mjlab`'s `deploy.yaml` into `policy.yaml` — gains, action scaling, default pose.
6. Translated the same file's observation list into `observations.yaml`, resolving two details (`cos_first`, `command_deadzone`) against the training environment's own source.
7. Fetched the ONNX model, validated its dimensions, and built and ran the controller.

## Going deeper

**[Reference](../reference/index.md)** is the complete, robot-agnostic documentation of everything touched above: the [architecture](../reference/architecture.md) behind it, every [controller](../reference/controller-configuration.md), [convention](../reference/conventions.md), [policy](../reference/policy-options.md), and [observation](../reference/observations.md) field, and [logging](../reference/logging.md) output.

## Adapting a different robot

Everything above specializes one general procedure to one specific robot and policy. If your next target isn't G1, here's that procedure on its own, with pointers back to where this tutorial worked through each part.

### 1. Fork and rename

Fork the template repository and name it after the new robot, then rename all integration points consistently: controller class/files, exported constructor, state classes and export names, CMake targets, configuration file, and GUI/logger prefixes.

```bash
sed -i -e 's/NewRLQPController/RobotNameRLQPController/g' $(find . -not -path '*/.*' -type f)
git mv src/NewRLQPController.cpp src/RobotNameRLQPController.cpp
git mv include/NewRLQPController.h include/RobotNameRLQPController.h
git mv src/states/NewRLQPController_Initial.cpp src/states/RobotNameRLQPController_Initial.cpp
git mv src/states/NewRLQPController_Initial.h src/states/RobotNameRLQPController_Initial.h
git mv etc/NewRLQPController.in.yaml etc/RobotNameRLQPController.in.yaml
```

You can also customize the project name in `vcpkg.json`, following the [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md).

### 2. Work through every `TODO(robot)`

```bash
grep -rn "TODO(robot)" .
```

In order, it points at:

1. **The controller configuration file** — the body used by floating-base observations, and the `ObserverPipeline`, matched to what your robot exposes and what your policy's observations need. See [Configure the controller](02-controller-configuration.md) and [Wire up state estimation](03-state-estimation.md), including the `Tilt` observer's `anchorFrameFunction` datastore callback.
2. **The controller's header** — the CBF gains (`zeta_jointLimit_`, `lambda_jointLimit_`, `zeta_selfCollision_`, `lambda_selfCollision_`), tuned for your robot's dynamics and safety margins.
3. **The controller's source file** — the selected constraints, the stiffness/weight, and contact/constraint consistency (surfaces, friction, collision pairs, and what your states expect). See [Adaptation advice: Controller tasks and constraints](../reference/adaptation-advice.md#controller-tasks-and-constraints) for both of the above two.
4. **`conventions.yaml`** — placeholder joint names, replaced with your own convention(s) in exactly the order your policy was trained/exported with. See [Define the training convention](04-training-convention.md).

### 3. Add your policies

See [Policy options](../reference/policy-options.md) and [Available observations](../reference/observations.md) for the full field reference, or [Configure the policy](05-policy-configuration.md) and [Configure the observations](06-observations-configuration.md) for this tutorial's worked example. If a policy needs an observation type this template doesn't implement yet, [Add an observation](../reference/add-observation.md) covers extending the registry.

### Elsewhere

[Adaptation advice](../reference/adaptation-advice.md) collects the recurring decisions (initial state or not, contact strategy, available sensors, control mode) across the robots this template has been adapted to, including the choices made for G1 above — existing robot variants (see the [project variants](../index.md#project-variants) on the home page) are useful references for seeing a given strategy applied in full.

## If something doesn't work

[Troubleshooting](../troubleshooting.md) covers the failure modes this kind of adaptation tends to hit — dimension mismatches, missing states, silent convention defaults, and more.
