---
title: Adaptation advice
---

# Adaptation advice

While the policy runtime is designed to be robot-agnostic, integrating a new robot often requires adapting parts of the controller to its hardware and control architecture.

This page highlights examples of adaptations encountered when integrating a new robot.

## Initial state

Some robots can safely start directly and immediately execute the RL policy.

Others require an initialization phase before enabling RL, for example to:

- reach a stable posture;
- initialize estimators;
- wait for operator validation;
- smoothly switch from another control mode.

In this case, consider introducing a dedicated `Initial` state before entering the RL state.

!!! example "Example"

    The HRP5P controller includes a dedicated `Initial` state before switching to RL, while the H1 and G1 controllers start directly in the RL state.

---

## Contact handling

Choose a contact model appropriate for the robot.

Typical options include:

- force sensor based contact detection;
- externally estimated contact forces;
- fixed or manually controlled contact states.

The selected strategy may also determine which observers and constraints are required.

!!! example "Example"

    The HRP5P implementation relies on force sensors, whereas the H1 implementation illustrates how to operate without dedicated foot force sensors.

---

## Available sensors

Not every robot provides the same measurements. Modifications of the ObserverPipeline and the observations implementations might be needed.

When configuring `observations.yaml`, only enable observations that can be computed from the available hardware.

Typical differences include:

- force sensors;
- IMU availability;
- floating-base estimation;
- external force estimation.

!!! tip

    The G1, H1, and HRP5P repositories provide examples of observers and observation configurations for robots with different sensing capabilities.

---

## Tilt observer

The `Tilt` observer estimates the robot orientation with respect to an anchor frame. It is usually necessary in this controller for floating-base robots.

Unlike most observations, enabling this observer also requires controller-side integration. The controller must register an `anchorFrameFunction` callback in the datastore before the observer is created.

The callback returns the anchor frame used as the tilt reference. Its implementation is robot-specific and may depend on the robot's contact model.

!!! warning

    Listing the `Tilt` observer in the controller configuration's `ObserverPipelines` is not sufficient on its own. The corresponding datastore callback must also be provided by the controller.

!!! example "Example"

    The H1 implementations demonstrates how to register the `anchorFrameFunction` callback and implement the robot-specific anchor frame computation. [The tutorial](../tutorial/03-state-estimation.md) walks through the same wiring for G1.

!!! tip "Observer availability"
    Some of the Observers used here are recent additions still living in forks or an open pull request rather than a release. If you have not used the proposed [superbuild](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild/tree/main), check what your own `mc_state_observation` install actually provides before relying on a given type being available.

---

## External Forces Observer

The `ExternalForcesObserver` estimates external contact forces and should be configured according to the sensing capabilities of the robot.

Robots equipped with force/torque sensors can directly exploit these measurements, whereas robots without dedicated force sensors generally rely on momentum-based estimation.

Important parameters include:

- `estimation_method`
- `torque_source_type`
- `residual_gain`
- `use_forces_from_ft_sensors`

These parameters directly depend on the robot hardware and should be reviewed when adapting the controller.

!!! example "Examples"

    The H1 and G1 implementations both illustrate a momentum-based configuration (`MomentumObserver`) without force/torque sensors, while the HRP5P implementation provides an example of a force-sensor-based configuration (`ForceSensorBased`).

For more precision, rely on [mc_external_forces_observer](https://github.com/isri-aist/mc_external_forces_observer)

---

## Controller tasks and constraints

The default task and constraint set is intended as a starting point.

Depending on the robot, additional components may be required, such as constraints or robot-specific mechanisms.

Adapt the task stack to the robot rather than trying to preserve the default configuration. This includes the CBF-QP gains (`zeta_jointLimit_`, `lambda_jointLimit_`, `zeta_selfCollision_`, `lambda_selfCollision_` in the controller header) and the stiffness/weight of the `TorqueJointTask` itself — see [the tutorial's closing checklist](../tutorial/08-next-steps.md#adapting-a-different-robot) for where these live in the source. The G1 controller currently runs with these left at the template's defaults; that they work doesn't by itself tell you whether they were deliberately checked or just never revisited — review them for your own robot either way.
