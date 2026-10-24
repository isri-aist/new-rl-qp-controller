---
title: Available observations
---

# Available observations

The default observation registry exposes ten internal observation types. If a needed observation is missing, it can be added : see [Add an observation](./add-observation.md).

## Observation pipeline

Each entry in `observations.yaml` becomes an `ObservationConfig`, an `Observation` subclass, and a history buffer managed by `ObservationManager`.

```yaml
training_convention: mjlab
history_order_newest_first: false

observations:
  - type: joint_pos
    name: joint_positions
    parameters:
      joints: all_no_grippers
      history: 5
      scale: 1.0
```

The final policy input is concatenated in the same order as the YAML list. For every term, its history samples are concatenated according to `history_order_newest_first`.

!!! warning "History order must reproduce training"
    `history_order_newest_first: true` concatenates the latest sample first. `false` places the oldest stored sample first. A correct dimension with the wrong temporal order can still produce completely invalid policy behavior.

## Common parameters

| Parameter | Meaning |
|---|---|
| `history` | Number of stored samples for this term. Must be positive. Default: `1`. |
| `name` | Optional unique/debug label. Defaults to the requested YAML type. |
| `scale` | Scalar or vector multiplier. A vector must match the observation size. |
| `joints` | A named `conventions.yaml` joint group, or an explicit inline list of joint names — the group is resolved first, falling back to the list. Using a convention group here is a convenience, not a requirement; see [Training conventions: is a convention required?](conventions.md#is-a-convention-required). |

Convention defaults are merged first, followed by local parameters from `observations.yaml`, so policy-local values override the convention.

## Observation reference

### `joint_pos`

Joint positions in the selected joint order.

| Property | Value |
|---|---|
| Size | Number of selected joints |
| Default source | `encoderValues()` when its size matches `refJointOrder()`; otherwise `mbc().q` |
| Parameters | `joints`, `relative_to_default_pose` (default `true`), `scale` |

When relative mode is enabled, the observation subtracts the corresponding `q_zero` value.

```yaml
- type: joint_pos
  parameters:
    joints: legs
    relative_to_default_pose: true
    scale: 1.0
```

### `joint_vel`

Joint velocities in the selected joint order.

| Property | Value |
|---|---|
| Size | Number of selected joints |
| Default source | `encoderVelocities()` when complete; otherwise `mbc().alpha` |
| Parameters | `joints`, `relative_to_default_velocity` (default `true`), `scale` |

The current default velocity is zero, so relative mode presently subtracts zero.

### `projected_gravity`

World gravity `[0, 0, -1]` projected into a configured robot body frame.

| Property | Value |
|---|---|
| Size | 3 |
| Parameters | `body` (defaults to `robot.base_body`), `scale` |

The named body must exist on the selected observation robot.

### `base_ang_vel`

Angular velocity ω of the base.

| Property | Value |
|---|---|
| Size | 3 |
| Parameters | `body` (default `base`) or `sensor`, `sensor_frame`, `frame` (default `local`), `scale` |

### `base_lin_vel`

Linear velocity v of the base origin.

| Property | Value |
|---|---|
| Size | 3 |
| Parameters | `body` (default `base`) or `sensor`, `sensor_frame`, `frame` (default `local`), `scale` |

**Source** (`base_ang_vel` and `base_lin_vel`)

| Parameter | Reads | Typical use |
|---|---|---|
| `body: base` (default) or `body: <name>` | ω and v of the body origin from the observed robot, written by whichever floating-base observer runs | real robot and simulation |
| `sensor: FloatingBase` | simulator ground truth filled by mc_mujoco, in the world frame | simulation only |
| `sensor: <IMU>` | the IMU gyro, ω in the sensor frame | `base_ang_vel` only |

An IMU does not measure v, so `base_lin_vel` with an IMU sensor raises an error. If your interface does fill a linear velocity, set `sensor_frame` explicitly to allow it.

`sensor_frame` (`world` or `local`) says how the sensor stores its velocity. It defaults to `world` for `FloatingBase` and to `local` for any other sensor.

**Output frame** (`frame`)

| Value | Meaning |
|---|---|
| `local` (default) | frame of the source body or sensor; what most locomotion policies use |
| `world` | world frame |
| `heading` | world frame rotated by the base yaw only: x forward and y left in the horizontal plane, z vertical |

The `world` and `heading` outputs need the source orientation. With `body`, it comes from the observer. With `sensor`, it comes from the sensor orientation, which the simulator, the IMU, or an observer (Tilt with `updateSensor`) must fill.

!!! warning "IMU mounting"
    With `sensor: <IMU>` and `frame: local`, ω is in the IMU frame. It equals the base frame only if the IMU is mounted without rotation on the base link.

### `last_action`

The previous raw ONNX action, reordered for the selected joint set.

| Property | Value |
|---|---|
| Size | Number of selected joints |
| Parameters | `joints`, `scale` |

The observation searches `policyJointControllerIndices` for each requested controller joint. If a selected joint does not exist in the policy action mapping, its value remains zero.

!!! note "Raw, not scaled"
    `last_action` uses the previous **raw policy output**, before applying per-joint action scales and before adding `q_zero`.

### `command`

The high-level velocity command vector held by `RLPolicyRuntime`.

| Property | Value |
|---|---|
| Size | 1 to 3; default `3` |
| Order | `vx`, `vy`, `yaw_rate` |
| Parameters | `size`, `scale` |

The GUI exposes all three values as editable number inputs.

### `base_orientation`

Roll, pitch, and yaw of the base.

| Property | Value |
|---|---|
| Default size | 3 |
| Parameters | `sensor` (default `Accelerometer`) or `body` (`base` or a body name), `index` (default `[0,1,2]`), `scale` |

With `sensor`, the orientation comes from the body sensor, so something must fill it: the simulator, the IMU, or an observer such as Tilt with `updateSensor`. With `body`, it comes from the observed robot, written by whichever floating-base observer runs, so it works the same with Tilt, MCWaiko or BodySensor.

`index` can select and reorder RPY components. For example, `[0,1]` returns roll and pitch.

!!! warning "Scale vector behavior"
    The implementation reads a three-element scale even when `index` selects fewer components. Use a scalar or a three-element vector.

### `phase`

A cyclic phase encoded as cosine and sine.

| Property | Value |
|---|---|
| Size | 2 |
| Parameters | `offset` (default `0`), `scale`, `cos_first` (default `true`) |

The runtime computes:

```text
phase_normalized = fmod(elapsed_time / phase_period, 1.0)
angle = 2π · (phase_normalized + offset)
```

With `cos_first: true`, output order is `[cos(angle), sin(angle)]`. With `false`, it is `[sin(angle), cos(angle)]`.

### `log_force_sensor`

Concatenated force vectors from one or more mc_rtc force sensors.

| Property | Value |
|---|---|
| Size | `3 × number_of_sensors` |
| Parameters | `sensor_names` (required, non-empty) |

For each sensor, the world-frame force is transformed component-wise with signed `log1p` compression:

```text
sign(f) · log(1 + |f|)
```

### `force_sensor`

Concatenated raw force vectors from one or more mc_rtc force sensors.

| Property | Value |
|---|---|
| Size | `3 × number_of_sensors` |
| Parameters | `sensor_names` (required, non-empty) |

## Registered internal names

```text
joint_pos
joint_vel
projected_gravity
base_ang_vel
base_lin_vel
base_orientation
phase
last_action
command
force_sensor
```

Unknown internal types fail with an error that also lists all known registry entries.

!!! tip "Unknown observation type"
    In the case a specific observation type is needed and missing, see [Add an observation](add-observation.md).

## Dimension validation

The runtime validates the complete concatenated observation size against the ONNX input size when loading a policy and again before inference. This catches missing terms and history-length mistakes, but it cannot detect semantically wrong ordering, scaling, frames, or aliases.
