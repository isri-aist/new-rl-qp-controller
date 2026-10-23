---
title: "1. Fork and rename"
---

# 1. Fork and rename

## Which branch? { #which-branch }

Fork [`new-rl-qp-controller`](https://github.com/isri-aist/new-rl-qp-controller), and work from its **`main` branch**. It is what this tutorial — and the [`g1_rl_qp_controller`](https://github.com/Alhuuin/g1_rl_qp_controller) repository it produces — was built from.

(Rename the clone's remote/folder to your robot's name as you see fit — the commands below don't depend on it.)

## Rename exported integration points

Rename all integration points consistently: controller class/files, exported constructor, state classes and export names, CMake targets, configuration file, GUI/logger prefixes, and test selection :

```bash
sed -i -e 's/NewRLQPController/G1RLQPController/g' $(find . -not -path '*/.*' -type f)
git mv src/NewRLQPController.cpp src/G1RLQPController.cpp
git mv include/NewRLQPController.h include/G1RLQPController.h
git mv src/states/NewRLQPController_Initial.cpp src/states/G1RLQPController_Initial.cpp
git mv src/states/NewRLQPController_Initial.h src/states/G1RLQPController_Initial.h
git mv etc/NewRLQPController.in.yaml etc/G1RLQPController.in.yaml
```

You can optionally also update the project name in `vcpkg.json`, following the [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md). It doesn't affect the controller's runtime behavior.

## Track the needed adaptations

The template tags every remaining robot-specific decision as `//TODO(robot)` in the source. Keep this handy through the rest of the tutorial, what it finds is resolved by the following steps.

```bash
grep -rn "TODO(robot)" .
```

## Expected architecture overview

After the rename, the tree looks like this (unchanged from the template otherwise):

```text
g1_rl_qp_controller/
├── etc/
│   └── G1RLQPController.in.yaml
│
├── include/
│   ├── G1RLQPController.h
│   ├── api.h
│   ├── observation/
│   │   ├── Observation.h
│   │   ├── ObservationManager.h
│   │   └── Observations.h
│   └── policy/
│       ├── PolicyConfig.h
│       ├── RLPolicyInterface.h
│       ├── RLPolicyRuntime.h
│       └── RLStateRunner.h
│
├── src/
│   ├── G1RLQPController.cpp
│   ├── lib.cpp
│   ├── observation/
│   ├── policy/
│   └── states/
│       ├── CMakeLists.txt
│       ├── G1RLQPController_Initial.cpp
│       └── G1RLQPController_Initial.h
│
├── policies/
│   ├── conventions.yaml
│   ├── minimalExample/
│   └── fullExample/
│
├── scripts/
│   └── inspect_policy.py
│
├── tests/
└── ext/onnxruntime
```

Nothing here runs yet — the controller has not yet been set up and no policy has been configured yet. That's what the rest of this tutorial fills in.
