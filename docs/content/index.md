---
title: RL QP Controller
---

<div class="hero" markdown>

# RL QP Controller

**Deploy ONNX reinforcement-learning policies inside an mc_rtc controller, with QP-based constraint enforcement.**

<div class="hero-actions" markdown>
[Tutorial](tutorial/index.md){ .md-button .md-button--primary }
[Reference](reference/index.md){ .md-button }
[Troubleshooting](./troubleshooting.md){ .md-button }
</div>

</div>

<!-- <div class="status-strip" markdown>
<span>Template-first design</span><span>ONNX inference</span><span>Convention-aware observations</span><span>Runtime policy switching</span>
</div> -->

## Project variants

The project is maintained as a reusable base template. **[The tutorial](tutorial/index.md) is the intended way to start a new integration**: it walks you through adapting the template end to end to a Unitree G1, and running an open-source policy. The same steps apply to any other robot and policy.

The adaptations below are what following that tutorial produces for a specific robot. If your target happens to match one of them closely - similar robot, similar policy — it can be useful to have a look at them as examples or inspirations.

<div class="repo-grid" markdown>

<div class="repo-card repo-card--featured" markdown>
### :material-source-repository: Base template

The robot-agnostic starting point, on its `main` branch. This is the repository the tutorial adapts.

[Open template repository](https://github.com/isri-aist/new-rl-qp-controller/tree/main){ .md-button .md-button--primary }
</div>

<div class="repo-card" markdown>
### :material-robot: Unitree G1

Built by following this documentation's tutorial: a Unitree G1 running an open-source velocity policy from Unitree.

[Open G1 repository](https://github.com/Alhuuin/g1_rl_qp_controller){ .md-button }
</div>

<div class="repo-card" markdown>
### :material-robot: HRP5P

The HRP5P adaptation, including its robot configuration, policies, controller-specific integration, and FSM states.
This adaptation is currently closed-source.

[HRP5P repository (closed-source)](https://github.com/bastien-muraccioli/hrp5p_rl_qp_controller){ .md-button }
</div>


<div class="repo-card" markdown>
### :material-robot: Unitree H1

The H1 adaptation, including its robot configuration, policies, and controller-specific integration.

[Open H1 repository](https://github.com/Alhuuin/h1_rl_qp_controller){ .md-button }
</div>

</div>

## What the controller does

At runtime, the controller:

1. searches for policy folders under `policies_root`;
2. for the selected policy, loads `policy.yaml`, `observations.yaml` and ONNX model;
3. resolves training-environment aliases and joint ordering through `conventions.yaml`;
4. constructs the policy input according to the observations configuration;
5. runs ONNX inference at the configured policy period;
6. maps and scales the action into controller joint order;
7. produces a target posture and PD torque command;
8. applies torque through the configured control path (`use_QP`).

```text
Robot state ──► ObservationManager ──► ONNX policy ──► action mapping
                    ▲                                      │
                    │ history + convention                  ▼
             observations.yaml                    q_rl = q0 + scale·action
                                                           │
                                      ┌────────────────────┴───────────────────┐
                                      ▼                                        ▼
                              TorqueJointTask + QP                     direct torque path
```


## Documentation map

- **[Tutorial](tutorial/index.md)** takes you step by step through adapting the template to a Unitree G1 and an open-source policy: forking and renaming, controller configuration, state estimation, training conventions, and both policy configuration files, down to a running controller.
- **[Reference](reference/index.md)** documents the full system architecture, every configuration file and field, available observations, and logging outputs — the pages the tutorial links to as it goes — as well as how to add custom observations and apply context-based integration advice when adapting further.
- **[Troubleshooting](troubleshooting.md)** covers common issues and solutions.
