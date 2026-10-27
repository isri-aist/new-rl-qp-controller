---
title: "6. Configure the observations"
---

# 6. Configure the observations

This step creates `policies/unitree/observations.yaml`. For the full reference, see [Available observations](../reference/observations.md) — this page derives the file term by term from the training environment.

## The observation list, in training order

`deploy.yaml`'s `observations` block (same file as the previous step) lists every term the policy was trained with, **in order** — this order is exactly what `observations.yaml`'s own list must reproduce, since [the final policy input is concatenated in YAML order](../reference/observations.md#observation-pipeline):

```yaml title="deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml (excerpt)"
observations:
  base_ang_vel: {...}
  projected_gravity: {...}
  velocity_commands: {params: {command_name: base_velocity}, ...}
  gait_phase: {params: {period: 0.6}, ...}
  joint_pos_rel: {...}
  joint_vel_rel: {...}
  last_action: {...}
```

## Term by term

| `deploy.yaml` name | internal type used | why |
|---|---|---|
| `base_ang_vel` | `base_ang_vel` (`sensor: FloatingBase`) | Same name. |
| `projected_gravity` | `projected_gravity` (`sensor: FloatingBase`) | Same name. |
| `velocity_commands` | `command` | This convention doesn't define a `type_aliases` entry for it (see [Define the training convention](04-training-convention.md#why-type_aliases-stays-empty-here)), so the internal name is used directly. |
| `gait_phase` | `phase` | `params.period: 0.6` becomes `control.phase_period` in `policy.yaml` (previous step), not a per-observation parameter. |
| `joint_pos_rel` | `joint_pos` (`relative_to_default_pose: true`) | The `_rel` suffix in mjlab means "relative to the default pose" — exactly what `relative_to_default_pose: true` does here. |
| `joint_vel_rel` | `joint_vel` (`relative_to_default_velocity: true`) | Same pattern for velocity. |
| `last_action` | `last_action` | Same name. |

### `phase`: `cos_first` and `command_deadzone`

`deploy.yaml` doesn't say what order `gait_phase` packs sine and cosine in, or explain the `0.1` behind the scenes — that's determined by the observation's actual implementation in the training environment, in [`unitree_rl_mjlab`'s `src/tasks/velocity/mdp/observations.py`](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/main/src/tasks/velocity/mdp/observations.py):

```python title="src/tasks/velocity/mdp/observations.py (excerpt)"
def phase(env: ManagerBasedRlEnv, period: float, command_name: str) -> torch.Tensor:
    global_phase = (env.episode_length_buf * env.step_dt) % period / period
    phase = torch.zeros(env.num_envs, 2, device=env.device)
    phase[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
    phase[:, 1] = torch.cos(global_phase * torch.pi * 2.0)
    stand_mask = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) < 0.1
    phase = torch.where(stand_mask.unsqueeze(1), torch.zeros_like(phase), phase)
    return phase
```

Two things this settles directly:

- The output is `[sin(phase), cos(phase)]` — sine first — hence `cos_first: false`.
- Whenever the commanded velocity's norm is below `0.1`, the training environment forces the phase to zero instead of letting it keep cycling — hence `command_deadzone: 0.1` on the `phase` observation (see [Available observations: `phase`](../reference/observations.md#phase)). Omitting this would have the robot see a walking-gait phase signal while being commanded to stand still, which never happened during training.

It is always a good habit to verify the implementation itself. Not every environment implement observations the same way.

### History

`history_order_newest_first: true` and `history: 1` on every term: this policy doesn't stack multiple past frames per observation — each term is just its current value. See [Available observations: common parameters](../reference/observations.md#common-parameters) if a future policy of yours does use `history > 1`.

## The complete file

```yaml title="policies/unitree/observations.yaml"
# Full observation configuration reference.
# Every top-level field and every parameter currently accepted by the built-in
# observations is represented.

training_convention: mjlab

# Optional. Defines the order in which to handle all observations history. Defaults to true.
history_order_newest_first: true

observations:
  - type: base_ang_vel
    parameters:
      history: 1
      sensor: FloatingBase
      scale: [1.0, 1.0, 1.0]

  - type: projected_gravity
    parameters:
      history: 1
      sensor: FloatingBase
      scale: [1.0, 1.0, 1.0]

  - type: command
    parameters:
      history: 1
      # Optional command size in [1, 3]. Defaults to 3.
      size: 3
      scale: [1.0, 1.0, 1.0]

  - type: phase # [cos(phase), sin(phase)] or [sin(phase), cos(phase)]
    parameters:
      history: 1
      # Optional normalized phase offset. Defaults to 0.0.
      offset: 0.0
      cos_first: false
      scale: [1.0, 1.0]
      command_deadzone: 0.1

  - type: joint_pos
    # Optional debug/log label. Defaults to type.
    parameters:
      # Optional. Defaults to 1 and must be positive.
      history: 1
      # Optional. May be a named group or an explicit list of joint names.
      joints: all # OR joints: [jointName1, jointName2, jointName3]
      # Optional. Defaults to true.
      relative_to_default_pose: true
      # Optional scalar or vector matching the observation size.
      scale: 1.0

  - type: joint_vel
    parameters:
      history: 1
      joints: all
      relative_to_default_velocity: true

  - type: last_action
    # Optional : Renaming of the term - useful in logs or to match the ONNX metadata observation_names
    name: actions
    parameters:
      history: 1
      joints: all
```

The `name: actions` matches the term name stored in the ONNX file, so the controller's cross-check of the observation order passes without a warning (see [Inspect the policy](07-running-and-validating.md#metadata)).

## Where you should be now

`policies/unitree/observations.yaml` exists and is complete. The folder is still missing one thing: the ONNX model itself. That's the next step, along with the final validation before running the controller.
