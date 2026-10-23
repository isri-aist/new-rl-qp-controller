---
title: Troubleshooting
---

# Troubleshooting

If this is your first adaptation, working through [the tutorial](tutorial/index.md) step by step prevents most of the issues below.

## Controller or FSM state is not found

Check the exported controller name in `src/lib.cpp`, selection in `mc_rtc.yaml`, `EXPORT_SINGLE_STATE` names, `src/states/CMakeLists.txt`, and state search paths. A source file may exist without being built.

## Policy folder is ignored

A policy folder must be directly below `policies_root` and contain `policy.yaml`, `observations.yaml`, and the resolved ONNX file. Remember that `@CMAKE_CURRENT_SOURCE_DIR@` is substituted at configure time.

## Default policy is unknown

The runtime name is `policy.yaml/name`, or the folder name when omitted. Names must be unique and `default_policy` must match the runtime name.

## ONNX input size mismatch

Compute:

```text
sum(observation base size x history)
```

Then verify aliases, convention defaults, joint-group sizes, `base_orientation.index`, `command.size`, force-sensor count, history, and repeated terms.

If neeeded, you can check the expected input/output sizes of the policy loaded with the helper script.

## ONNX action size mismatch

The output size must equal resolved `action.joints`, not `controlled_joints`. Check the logged controller indices.

## Policy loads but behavior is wrong

Observation dimensions alone do not guarantee semantic equivalence. Verify that the observation order, joint order, reference frames, sign conventions, scaling, relative vs. absolute values, history ordering, phase period and reset behavior, command ordering, action scaling, and default pose all match those used during training.

Also verify the conventions in use, ensuring that no aliases or default parameters interfere with the intended observation construction.

Review the implementation of each observation to confirm that it matches the expected behavior exactly.

Finally, verify the observers used by the controller and ensure that they provide consistent and coherent information.

Use `mc_log_ui` to display the logs.

## Body, body sensor, or force sensor is missing

These are different resources and APIs. Confirm the exact robot-module names and whether the observation expects a body, body sensor, or force sensor.

## Tilt observer fails

Verify `anchorFrameFunction` is registered before use, the datastore key matches, `createContactAnchor()` resolves required resources, and measured state is valid. See context-based patterns in [Adaptation advice](reference/adaptation-advice.md), or the tutorial example in [Wire up state estimation](tutorial/03-state-estimation.md).

## Template examples do not run

It is a template, meant to be adapted to a specific robot. The policy configurations intentionally contain placeholder joints and zero-byte placeholder ONNX files. See [the tutorial](tutorial/index.md) for a full worked example, or its closing [Adapting a different robot](tutorial/08-next-steps.md#adapting-a-different-robot) section for the generalized procedure.
