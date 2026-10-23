---
title: Reference
---

# Reference

This section documents the controller in depth: every configuration file, every field, and the runtime behavior behind them. It's the material [the tutorial](../tutorial/index.md) links to as it walks through adapting the template.

## Controller design

The project has two practical layers:

- **mc_rtc integration**: controller/FSM, tasks, constraints, observers, states, transitions. See [Architecture](architecture.md).
- **policy runtime**: policy loading, convention resolution, observation assembly, ONNX inference, action mapping. See [Architecture](architecture.md) for the full object graph, and the pages below for each configuration surface.

## Configuration files

The project separates configuration by scope so each file has one clear role.

| Scope | File | Covers | Reference |
|---|---|---|---|
| Controller-level | `etc/<Controller>.in.yaml` | Robot-level and runtime integration settings: observation source, base body, observer pipeline, policies root, default policy. | [Controller configuration](controller-configuration.md) |
| Policy folder | `<policies_root>/<policy>/{policy,observations}.yaml` + ONNX | `policy.yaml` defines action mapping and control parameters; `observations.yaml` defines policy input terms, order, and history. | [Policy options](policy-options.md), [Available observations](observations.md) |
| Shared convention | `<policies_root>/conventions.yaml` | Training-environment conventions: joint groups and order, optional type aliases and default observation parameters. Every policy depends on it. | [Training conventions](conventions.md) |

## Policy loading

Policies are loaded from the configured policy base directory. By default, this directory is `policies/`, but it can be changed in the controller configuration file. Every valid policy subdirectory found there is loaded and can be selected at runtime through the mc_rtc GUI. See [Controller configuration](controller-configuration.md#2-point-to-the-policy-directory).

## Observation history

For each observation (joint velocity, last action, …), a `history` can be specified, corresponding to the number of past values of this observation that the policy needs.

Observation history is concatenated by observation, newest-first by default. Ensure that this matches the ordering used during training — see [Available observations](observations.md#observation-pipeline) for the full semantics.

## Logging

At runtime, every relevant quantity — gains, targets, the assembled observation vector, the active policy and convention — is exposed to `mc_log_ui` and the GUI. See [Logging](logging.md).

## Extending and generalizing

The tutorial builds one concrete controller, for one robot and one policy. These pages generalize past that:

- **[Add an observation](add-observation.md)** — implement and register a new observation type when the built-in registry is missing one your policy needs.
- **[Adaptation advice](adaptation-advice.md)** — recurring integration decisions (initial state, contact handling, available sensors, control mode) and how the robots this template has been adapted to so far each resolved them.

For adapting the template to a robot other than G1, see the tutorial's own [Adapting a different robot](../tutorial/08-next-steps.md#adapting-a-different-robot) — the same procedure the tutorial just walked through for G1, generalized, with pointers back into each worked step.
