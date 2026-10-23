---
title: Tutorial
---

# Tutorial: adapting the template to a Unitree G1

This tutorial adapts the base template, step by step, to a concrete robot and policy. Here we take the example of :

- **Robot**: Unitree G1 (29 actuated joints).
- **Policy**: the open-source velocity-tracking policy exported by [`unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab).

By the end, you will have a controller that loads this policy, runs it through the QP, and can be commanded with a walking velocity command.

!!! tip "Comparing against a finished result"
    Following this tutorial, step by step, produces the [`g1_rl_qp_controller`](https://github.com/Alhuuin/g1_rl_qp_controller) repository.

## Requirements

- The [safe-rl-qp superbuild](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild), which builds and installs mc_rtc and every dependency used here (see below).
- `github.com/unitreerobotics/unitree_rl_mjlab`, to fetch the policy's configuration and ONNX model.

ONNX Runtime does not need to be installed: the template ships its own copy in `ext/onnxruntime` (see [Architecture](../reference/architecture.md#project-structure)).

### Setting up the environment

The superbuild clones, builds and installs, in one go, mc_rtc and the projects this tutorial relies on: the `mc_state_observation` fork that provides the `Tilt` observer, `mc_external_forces_observer`, `mc_joystick_plugin` (included by the controller), `mc_mujoco` for simulation, and — with the `WITH_G1` option — the G1 robot module (`mc_g1`) and its description. It is only tested on Ubuntu 22.04 and 24.04.

Follow the [superbuild README](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild#installation) for the full installation (fork it, run the bootstrap script, build). The one G1-specific step is to turn the robot on in the superbuild configuration, then rebuild:

```bash
cd ~/workspace/build/superbuild
ccmake .        # set WITH_G1 to ON, then [c] configure, [g] generate, [q] quit
cd ~/workspace/safe-rl-qp-mc-rtc-superbuild
cmake --build --preset relwithdebinfo
```

## Steps

1. [Fork and rename](01-fork-and-rename.md) the template for G1.
2. [Configure the controller](02-controller-configuration.md): tell the controller what it needs to know about your robot.
3. [Wire up state estimation](03-state-estimation.md): give the controller a reliable view of the robot's state.
4. [Define the training convention](04-training-convention.md): an optional but useful tool for describing a robot and training setup — here, mainly G1's joint order.
5. [Configure the policy](05-policy-configuration.md): how the policy's output drives the robot.
6. [Configure the observations](06-observations-configuration.md): the policy input, in the order seen during training.
7. [Run and validate](07-running-and-validating.md): fetch the ONNX model, check it against the configuration, build, and run.
8. [Next steps](08-next-steps.md): where to go from here.

Each step links back to the [Reference](../reference/index.md) page that documents the corresponding file or mechanism in full generality — this tutorial shows one concrete, working instance of it.
