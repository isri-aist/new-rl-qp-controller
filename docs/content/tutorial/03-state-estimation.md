---
title: "3. Wire up state estimation"
---

# 3. Wire up state estimation

The previous step enabled the `Tilt` observer in `etc/G1RLQPController.in.yaml`. As the previous step stated, that alone isn't enough: `Tilt` needs an anchor frame — a reference frame it can trust, typically wherever the robot currently has ground contact — and that anchor is robot-specific. The controller must provide it as a callback registered in the mc_rtc datastore under the key referenced in the config (`anchorFrameFunction`).

This step adds that callback to `G1RLQPController`.

## 1. Declare the method and its state

In `include/G1RLQPController.h`, declare the callback's implementation as a public method (it needs to be reachable from the constructor's datastore registration), and add a member to hold the anchor transform between calls:

```cpp title="include/G1RLQPController.h"
  //...

  std::pair<sva::PTransformd, Eigen::Vector3d> createContactAnchor(const mc_rbdyn::Robot & anchorRobot);

private:
  mc_rtc::Configuration config_;
  // ...
```

and, alongside the other private runtime members:

```cpp title="include/G1RLQPController.h"
  //...

  // Anchor frame for tilt estimation
  sva::PTransformd contactAnchorTf_;
```

The observer's `anchorFrameFunction` callback signature is `std::pair<sva::PTransformd, Eigen::Vector3d>(const mc_rbdyn::Robot &)`: a transform (the anchor frame) and a velocity (the anchor's linear velocity).

## 2. Implement the anchor computation

What the anchor *should* be is robot-specific — see [Adaptation advice: Contact handling](../reference/adaptation-advice.md#contact-handling) for the space of options. For robot that have no dedicated foot force/torque sensors (the same reason the previous step used `MomentumObserver` for external forces), this implementation is a proxy for weight distribution: the *external knee torque*, estimated by the same momentum observer, on each leg. The anchor is then the position (and velocity) of whichever foot is estimated to be more loaded, interpolated between both feet when the load is close to even:

```cpp title="src/G1RLQPController.cpp"
std::pair<sva::PTransformd, Eigen::Vector3d> G1RLQPController::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  sva::PTransformd X_foot_r = anchorRobot.bodyPosW("right_ankle_roll_link");
  sva::PTransformd X_foot_l = anchorRobot.bodyPosW("left_ankle_roll_link");

  sva::MotionVecd v_foot_r = anchorRobot.bodyVelW("right_ankle_roll_link");
  sva::MotionVecd v_foot_l = anchorRobot.bodyVelW("left_ankle_roll_link");

  int right_knee_index = int(robot().jointIndexByName("right_knee_joint")) + 5;
  int left_knee_index = int(robot().jointIndexByName("left_knee_joint")) + 5;
  double tau_ext_knee_r =  abs(robot().externalTorques()[right_knee_index]);
  double tau_ext_knee_l =  abs(robot().externalTorques()[left_knee_index]);
  double leftFootRatio = tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
  if(tau_ext_knee_r + tau_ext_knee_l < 0.02)
  {
    leftFootRatio = 0.5;
  }

  Eigen::VectorXd w_r = X_foot_r.translation();
  Eigen::VectorXd w_l = X_foot_l.translation();
  Eigen::VectorXd contact_anchor = (w_r * (1 - leftFootRatio) + w_l * leftFootRatio);
  Eigen::VectorXd anchor_vel = (v_foot_r.linear() * (1 - leftFootRatio) + v_foot_l.linear() * leftFootRatio);
  contactAnchorTf_ = sva::PTransformd(Eigen::Matrix3d::Identity(), contact_anchor);

  return {contactAnchorTf_, anchor_vel};
}
```

A few things worth noting if you're adapting this to a different robot rather than copying it as-is:

- `"right_ankle_roll_link"` / `"left_ankle_roll_link"` are body names from the G1 mc_rtc robot module — look up your own robot's foot/end-effector body names (`robot().bodies()`) rather than assuming these.
- The `+ 5` offset into `robot().externalTorques()` accounts for the 6 floating-base DoF preceding the actuated joints in that vector on this robot module (minus 1, since indexing starts differently) — this offset is specific to how your robot module lays out its generalized coordinates; verify it against your own module rather than reusing the number directly.
- The `0.02` threshold and the 50/50 fallback are a simple guard against dividing by a near-zero torque sum (e.g. when the robot is unloaded or momentarily in double support with near-equal loading), not a value derived from training or hardware specs. This is a reasonable starting point, not a tuned constant — revisit it if the anchor estimate looks noisy in practice.

This mirrors the pattern the H1 adaptation uses for the same purpose.

## 3. Register the callback

In `src/G1RLQPController.cpp`, `initializeRobotBasics()` already runs once at controller construction. Register the callback there, guarded so it's only made once:

```cpp title="src/G1RLQPController.cpp"
void G1RLQPController::initializeRobotBasics()
{
  // ... existing content ...

  if(!datastore().has("anchorFrameFunction"))
  {
    datastore().make_call("anchorFrameFunction", [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});
  }
}
```

## Other `TODO(robot)` markers in this file { #other-todorobot-markers-in-this-file }

`G1RLQPController`'s constructor — a few lines above the `initializeRobotBasics()` call edited above — carries two more `TODO(robot)` markers this tutorial doesn't touch:

```cpp title="src/G1RLQPController.cpp (constructor, unmodified from the template)"
//Initialize Constraints
// TODO(robot): review which constraints are valid for the selected robot.
// These defaults provide joint-limit and self-collision protection
selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
solver().removeConstraintSet(dynamicsConstraint);
dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
  new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
solver().addConstraintSet(dynamicsConstraint);

// Remove the default posture task created by the FSM
solver().removeTask(getPostureTask(robot().name()));
// Initialize Task
torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
    solver(), robot().robotIndex(), 100.0, 1); // TODO(robot): tune stiffness/weight
```

`zeta_jointLimit_`, `lambda_jointLimit_`, `zeta_selfCollision_`, and `lambda_selfCollision_` are the CBF gains declared in `include/G1RLQPController.h`; `100.0, 1` are the `TorqueJointTask`'s stiffness and solver weight.

The values present in the G1 controller work correctly, the TODO can be removed here. They were kept in this tutorial to highlight that it is part of the adaptation for another robot.

For your own robot, confirm the constraint set matches what your robot actually needs, and that the CBF gains and `TorqueJointTask` stiffness/weight suit its dynamics, then remove the marker once you have.

## Where you should be now

`grep -rn "TODO(robot)" .` now shouldn't report any markers in this file. Everything about the controller configuration and its state estimation is complete. What remains is entirely in `policies/`: the policy itself, its configuration, and optional conventions for smoother configurations.
