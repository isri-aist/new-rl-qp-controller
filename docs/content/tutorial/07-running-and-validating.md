---
title: "7. Run and validate"
---

# 7. Run and validate

## Fetch the ONNX model

Download the exported policy from `unitree_rl_mjlab` into the policy folder:

```bash
curl -L -o policies/unitree/policy.onnx \
  https://raw.githubusercontent.com/unitreerobotics/unitree_rl_mjlab/main/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx
```

This matches the `onnx: policy.onnx` filename set in `policy.yaml`.

!!! tip "Cleaning up the template's placeholder policies"
    `policies/minimalExample/` and `policies/fullExample/` were the template's placeholder policies (zero-byte ONNX files, generic joint names). They're not needed anymore now that `policies/unitree/` is a real policy — you can remove them, or keep them around as a reference for the file format. The `g1_rl_qp_controller` repository removes them.

## Optional - Inspect the policy

Before building, or in case of a dimension mismatch error, check the ONNX model against what was just configured. The template ships a helper that prints the input/output sizes and the training metadata stored in the file:

```bash
python3 scripts/inspect_policy.py policies/unitree/policy.onnx
```

The script imports the `onnxruntime` Python package, which is separate from the C++ ONNX Runtime bundled with the template: install it with `pip install onnxruntime` if it isn't already available in your Python environment. Add `--raw` to print long metadata values in full.

### Dimensions

You're checking it against the sizes implied by the previous two steps:

| Observation | Size |
|---|---:|
| `base_ang_vel` | 3 |
| `projected_gravity` | 3 |
| `command` | 3 |
| `phase` | 2 |
| `joint_pos` (29 joints × history 1) | 29 |
| `joint_vel` (29 joints × history 1) | 29 |
| `last_action` (29 joints × history 1) | 29 |
| **Total input** | **98** |

| Action | Size |
|---|---:|
| `action.joints: all` (29 joints) | 29 |

`inspect_policy.py`'s reported input and output shapes should read `98` and `29`. If they don't, work back through [Configure the observations](06-observations-configuration.md) and [Configure the policy](05-policy-configuration.md) — a mismatch here almost always means a term was skipped, an `all` group doesn't actually have 29 entries, or a `history` value doesn't match what's expected.

The runtime performs this same check itself when loading and before every inference — see [Troubleshooting: ONNX input/action size mismatch](../troubleshooting.md#onnx-input-size-mismatch) if it fails at that point instead.

### Metadata

The `Metadata` section shows the training data the exporter stored in the file, as the controller will use it (see [ONNX metadata](../reference/policy-options.md#onnx-metadata)):

- **Used by the controller**: each key the controller reads, and whether it is present, usable, or ignored (for example a list whose length doesn't match `joint_names`).
- **Joint order**: the `all` group, with the per-joint kp, kd, q0 and action scale. Compare it with the `all` group of your convention, if you defined one: the controller refuses to load if they differ.
- **Observation terms**: the input order. It should match the `observations.yaml` list entry for entry.
- **Other keys**: everything else the exporter stored, for information.

For the `unitree` policy, the 29 joints and the 7 terms `base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel, actions` are reported. `actions` is the `last_action` entry, which is why that entry sets `name: actions`. If the file has no metadata, the section says so: `policy.yaml` must then give every gain, and the convention the `all` group.

## Build

### Option 1 - with the superbuild

Register the controller in the [superbuild](index.md#setting-up-the-environment) with an extension file, then rebuild. Following the [superbuild README](https://github.com/safe-rl-qp/safe-rl-qp-mc-rtc-superbuild#add-your-controller-to-the-superbuild):

```cmake title="safe-rl-qp-mc-rtc-superbuild/extensions/g1_rl_qp_controller.cmake"
AddProject(g1_rl_qp_controller
  GITHUB {github username}/g1_rl_qp_controller
  GIT_TAG origin/{branch name}
  DEPENDS mc_rtc
)
```

```bash
cd ~/workspace/safe-rl-qp-mc-rtc-superbuild
cmake --build --preset relwithdebinfo
```

(`GITHUB` clones over HTTPS; use `GITHUB_PRIVATE` to clone over SSH.)

### Option 2 - by itself

To build the controller on its own instead, from the controller's folder:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j
sudo cmake --install .
```

Here, it will not be detected by the superbuild.

## Run

Select the controller in `~/.config/mc_rtc/mc_rtc.yaml`:

```yaml
MainRobot: G1
Enabled: G1RLQPController
```

`G1` is one of the names exported by the `mc_g1` robot module (`G1`, `G1_23dof`, `G1_29dof`). Then start it in simulation, for instance with `mc_mujoco --sync`, or on the robot with its driver. On startup:

- The GUI's **Policy** panel should show `unitree` selected as the current policy, with dimensions `98`/`29`.
- The **Command** panel exposes `vx`, `vy`, and `yaw_rate` — these are what drives the `command` observation. Start at zero and increase gradually.
- The **Control** panel's QP toggle should be on (`use_QP: true`, as configured).

See [Logging](../reference/logging.md) for what's available in `mc_log_ui` if you need to dig into the policy's actual inputs/outputs at runtime. For anything not covered above: [Training conventions §Validation checklist](../reference/conventions.md#validation-checklist) for convention/joint-order checks, and [Controller configuration §Validation checklist](../reference/controller-configuration.md#validation-checklist) for the observer/observation-source side.

## Where you should be now

A complete `G1RLQPController` that loads the `unitree` policy by default, runs it through the CBF-QP torque path, and can be commanded from the GUI and keyboard. For game controller control, the [mc_joystick_plugin](https://github.com/bastien-muraccioli/mc_joystick_plugin) can be installed. This is the same end state as [`g1_rl_qp_controller`](https://github.com/Alhuuin/g1_rl_qp_controller) — worth diffing your work against it if anything above didn't go as expected.
