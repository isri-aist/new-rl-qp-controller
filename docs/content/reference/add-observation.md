---
title: Add an observation
---

# Add an observation

The observation framework separates configuration, implementation, and registration.

## 1. Declare the class

Add a subclass to `include/observation/Observations.h`.

```cpp
/** @brief Describe the quantity, frame, ordering, any relevant information. */
class MyObservation : public Observation
{
public:
  MyObservation(const ObservationConfig &, const ObservationConvention &);

  void configure(const ObservationContext & context) override;
  int size() const override { return size_; }
  void compute(const ObservationContext & context,
               Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  int size_ = 0;
};
```

`configure()` should resolve parameters and validate robot resources. `compute()` should perform only the per-cycle computation.

## 2. Implement it

Add the implementation to `src/observation/Observations.cpp`.

Use:

- `observedRobot(context)` for the robot selected by the entry's `observation_source` (do not read `context.realRobot` or `context.controlRobot` directly);
- `context.controllerJointOrder` for canonical controller order;
- `context.policyJointControllerIndices` for policy action order;
- `context.qZeroControllerOrder` for the active default pose;
- `context.command` and `context.phaseNormalized` for runtime signals;
- `context.convention.resolveObservationParameters(...)` to merge convention defaults and local overrides;
- `readScale(...)` for scalar-or-vector scale handling.

!!! warning "Validate dimensions in `configure()`"
    A wrong `size()` affects history allocation and the final ONNX input. Fail early when a parameter vector, index list, sensor, body, or joint selector is inconsistent.

## 3. Register the type

Add the implementation name to `makeDefaultObservationRegistry()` in `src/observation/Observation.cpp`:

```cpp
registry.registerType("my_observation", &makeObservation<MyObservation>);
```

It can then be used directly:

```yaml
- type: my_observation
  parameters: { ... }
```

## 4. Add optional aliases or default parameters

```yaml
type_aliases:
  training_name: my_observation

default_parameters:
  training_name:
    scale: 0.5
```
