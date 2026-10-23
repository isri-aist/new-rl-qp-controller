---
title: "2. Configure the controller"
---

# 2. Configure the controller

This step edits `etc/G1RLQPController.in.yaml`, the usual mc_rtc controller configuration file. For the general meaning of each field, see the [Controller configuration reference](../reference/controller-configuration.md) — this page is the concrete fill-in for G1.

## Base body and observation source

```yaml
robot:
  base_body: pelvis
  observation_source: realRobot
```

This is not native to mc_rtc but specific to this controller :

- `base_body` is the default body used by floating-base observations (example : see [`projected_gravity`](../reference/observations.md#projected_gravity)) when they aren't given an explicit body or sensor. For G1's mc_rtc robot module, that's `pelvis`. The template's placeholder was `base_link` — check what your own robot module actually calls its floating-base body (`robot().mb().body(0).name()` or the module's documentation).

- `observation_source: realRobot` is the correct default for hardware deployment: observations are computed from measured state, not the control robot's target state. It is left unchanged here.

## Observer pipeline

`Encoder` is the template's default. Three more observers are added for G1:

```yaml
ObserverPipelines:
  name: MainPipeline
  gui: true
  log: true
  observers:
    - type: Encoder
      config:
        velocity: encoderVelocities
    - type: BodySensor
      update: true
      bodySensor: FloatingBase
    - type: Tilt
      required: true
      update: true
      config:
        anchorFrameFunction: anchorFrameFunction
        filterGains:
          initAlpha: 2
          initBeta: 2
          initGamma: 1
          finalAlpha: 2
          finalBeta: 2
    - type: ExternalForcesObserver
      estimation_method: MomentumObserver
      use_forces_from_ft_sensors: false
      residual_gain: 100
      torque_source_type: CommandedTorque
```

What each one provides:

- **`Encoder`**: joint positions and velocities.
- **`BodySensor`**: floating-base pose and velocity, here computed from the `FloatingBase` body sensor.
- **`Tilt`**: floating-base pose and velocity, estimated from the IMU and an anchor frame (provided by the C++ callback of the [next step](03-state-estimation.md)). It also writes its orientation estimate into the `FloatingBase` sensor. `filterGains` are the estimator's gains.
- **`ExternalForcesObserver`**: external torques on the joints, here estimated with a momentum observer (no force/torque sensors).

See [Adaptation advice](../reference/adaptation-advice.md) for the alternatives to these choices on other robots.

## Default policy

```yaml
policies_root: "@CMAKE_CURRENT_SOURCE_DIR@/policies"
default_policy: unitree
```

`policies_root` is unchanged. `default_policy` is set to `unitree` — [Configure the policy](05-policy-configuration.md) and [Configure the observations](06-observations-configuration.md) will set one up with that name in the next steps. It doesn't exist yet at this point in the tutorial; the controller won't start until it does.

## Full file so far

```yaml
---
Managed: false
StepByStep: true
IdleKeepState: false

StatesLibraries:
  - "@MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX@"
  - "@MC_STATES_RUNTIME_INSTALL_PREFIX@"
StatesFiles:
  - "@MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX@/data"
  - "@MC_STATES_RUNTIME_INSTALL_PREFIX@/data"
VerboseStateFactory: false

robots:
  ground:
    module: env/ground

robot:
  base_body: pelvis
  # realRobot is usually appropriate on hardware; robot can be useful in simulation.
  observation_source: realRobot

collisions:
  - type: collision
    useMinimal: true

# Constraint examples. Keep only constraints supported by your robot and application.
ObserverPipelines:
  name: MainPipeline
  gui: true
  log: true
  observers:
    - type: Encoder
      config:
        velocity: encoderVelocities
    - type: BodySensor
      update: true
      bodySensor: FloatingBase
    - type: Tilt
      required: true
      update: true
      config:
        anchorFrameFunction: anchorFrameFunction
        filterGains:
          initAlpha: 2
          initBeta: 2
          initGamma: 1
          finalAlpha: 2
          finalBeta: 2
    - type: ExternalForcesObserver
      estimation_method: MomentumObserver
      use_forces_from_ft_sensors: false
      residual_gain: 100
      torque_source_type: CommandedTorque

states: {}
transitions:
  - [G1RLQPController_Initial, OK, G1RLQPController_Initial, Strict]
init: G1RLQPController_Initial

policies_root: "@CMAKE_CURRENT_SOURCE_DIR@/policies"
default_policy: unitree
```
