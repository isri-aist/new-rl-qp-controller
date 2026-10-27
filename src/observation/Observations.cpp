#include "observation/Observations.h"


#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/rpy_utils.h>
#include <numeric>

#include <mc_rtc/logging.h>

#include <cmath>

namespace rlqp
{

//============================================================================//
// JointPosObservation
//============================================================================//

JointPosObservation::JointPosObservation(const ObservationConfig & config,
                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void JointPosObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::vector<int> controllerIndices = context.convention.resolveJointControllerIndices(
      parameters, context.controllerJointOrder, context.policyJointControllerIndices);

  const int n = static_cast<int>(controllerIndices.size());
  controllerIndices_ = controllerIndices;
  mbcIndices_.resize(static_cast<size_t>(n));
  defaultPose_ = Eigen::VectorXd::Zero(n);

  for(int i = 0; i < n; ++i)
  {
    const int ctrlIdx = controllerIndices[static_cast<size_t>(i)];
    const std::string & jointName = context.controllerJointOrder[static_cast<size_t>(ctrlIdx)];
    mbcIndices_[static_cast<size_t>(i)] = observedRobot(context).jointIndexByName(jointName);
    defaultPose_(i) = context.qZeroControllerOrder(ctrlIdx);
  }

  relativeToDefaultPose_ = readParameter<bool>(parameters, "relative_to_default_pose", true);
  scale_ = readScale(parameters, "scale", n, 1.0);
}

void JointPosObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & currentPos = observedRobot(context).encoderValues();
  const bool useEncoders = currentPos.size() == context.controllerJointOrder.size();

  if(!currentPos.empty() && !useEncoders)
  {
    mc_rtc::log::error_and_throw("[Observation:{}] encoderValues has size {}, expected {} or 0", name(),
                                 currentPos.size(), context.controllerJointOrder.size());
  }

  for(size_t i = 0; i < controllerIndices_.size(); ++i)
  {
    double value = 0.0;
    if(useEncoders)
    {
      value = currentPos[static_cast<size_t>(controllerIndices_[i])];
    }
    else
    {
      const auto & q = observedRobot(context).mbc().q[static_cast<size_t>(mbcIndices_[i])];
      if(q.size() != 1)
      {
        mc_rtc::log::error_and_throw("[Observation:{}] Joint '{}' has {} position DoFs, expected 1", name(),
                                     context.controllerJointOrder[static_cast<size_t>(controllerIndices_[i])],
                                     q.size());
      }
      value = q[0];
    }

    if(relativeToDefaultPose_) value -= defaultPose_(static_cast<int>(i));

    out(static_cast<int>(i)) = scale_(static_cast<int>(i)) * value;
  }
}

//============================================================================//
// JointVelObservation
//============================================================================//

JointVelObservation::JointVelObservation(const ObservationConfig & config,
                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void JointVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::vector<int> controllerIndices = context.convention.resolveJointControllerIndices(
      parameters, context.controllerJointOrder, context.policyJointControllerIndices);

  const int n = static_cast<int>(controllerIndices.size());
  controllerIndices_ = controllerIndices;
  mbcIndices_.resize(static_cast<size_t>(n));
  defaultVelocity_ = Eigen::VectorXd::Zero(n);

  for(int i = 0; i < n; ++i)
  {
    const int ctrlIdx = controllerIndices[static_cast<size_t>(i)];
    const std::string & jointName = context.controllerJointOrder[static_cast<size_t>(ctrlIdx)];
    mbcIndices_[static_cast<size_t>(i)] = observedRobot(context).jointIndexByName(jointName);
  }

  relativeToDefaultVelocity_ = readParameter<bool>(parameters, "relative_to_default_velocity", true);
  scale_ = readScale(parameters, "scale", n, 1.0);
}

void JointVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & currentVel = observedRobot(context).encoderVelocities();
  const bool useEncoders = currentVel.size() == context.controllerJointOrder.size();

  if(!currentVel.empty() && !useEncoders)
  {
    mc_rtc::log::error_and_throw("[Observation:{}] encoderVelocities has size {}, expected {} or 0", name(),
                                 currentVel.size(), context.controllerJointOrder.size());
  }

  for(size_t i = 0; i < controllerIndices_.size(); ++i)
  {
    double value = 0.0;
    if(useEncoders)
    {
      value = currentVel[static_cast<size_t>(controllerIndices_[i])];
    }
    else
    {
      const auto & alpha = observedRobot(context).mbc().alpha[static_cast<size_t>(mbcIndices_[i])];
      if(alpha.size() != 1)
      {
        mc_rtc::log::error_and_throw("[Observation:{}] Joint '{}' has {} velocity DoFs, expected 1", name(),
                                     context.controllerJointOrder[static_cast<size_t>(controllerIndices_[i])],
                                     alpha.size());
      }
      value = alpha[0];
    }

    if(relativeToDefaultVelocity_) value -= defaultVelocity_(static_cast<int>(i));

    out(static_cast<int>(i)) = scale_(static_cast<int>(i)) * value;
  }
}

//============================================================================//
// ProjectedGravityObservation
//============================================================================//

ProjectedGravityObservation::ProjectedGravityObservation(const ObservationConfig & config,
                                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void ProjectedGravityObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::string sensor = readParameter<std::string>(parameters, "sensor", "");
  const std::string body = readParameter<std::string>(parameters, "body", "");
  if (!sensor.empty() && !body.empty())
    mc_rtc::log::error_and_throw("[Observation:{}] Choose either 'sensor' or 'body' as source, not both", name());

  if(!sensor.empty())
  {
    useSensor_ = true;
    sensorName_ = sensor;
    if(!observedRobot(context).hasBodySensor(sensorName_))
      mc_rtc::log::error_and_throw("[Observation:{}] Body sensor '{}' does not exist on robot '{}'", name(), sensorName_, observedRobot(context).name());
  }
  else
  {
    const std::string obs_body = body.empty() ? context.baseBody : body;
    if(!observedRobot(context).hasBody(obs_body))
      mc_rtc::log::error_and_throw("[Observation:{}] Body '{}' does not exist on robot '{}'", name(), obs_body, observedRobot(context).name());
    bodyIndex_ = observedRobot(context).mb().bodyIndexByName(obs_body);
  }
  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void ProjectedGravityObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::Vector3d gravityWorld(0.0, 0.0, -1.0);
  Eigen::Vector3d gravityBody;

  if(useSensor_)
  {
    const auto & imu = observedRobot(context).bodySensor(sensorName_);
    gravityBody = imu.orientation().normalized().toRotationMatrix() * gravityWorld;
  }
  else
  {
    const sva::PTransformd & X_0_body =
      observedRobot(context).mbc().bodyPosW[static_cast<size_t>(bodyIndex_)];
    gravityBody = X_0_body.rotation() * gravityWorld;
  }

  out = gravityBody.cwiseProduct(scale_);
}

//============================================================================//
// VelocitySource (shared by base_ang_vel and base_lin_vel)
//============================================================================//

void VelocitySource::configure(const std::string & obsName,
                               const mc_rtc::Configuration & parameters,
                               const mc_rbdyn::Robot & robot,
                               const std::string & baseBody,
                               bool angular)
{
  const std::string sensor = parameters("sensor", std::string(""));
  const std::string body = parameters("body", std::string(""));
  if(!sensor.empty() && !body.empty())
    mc_rtc::log::error_and_throw("[Observation:{}] Choose either 'sensor' or 'body' as source, not both", obsName);

  if(!sensor.empty())
  {
    useSensor = true;
    sensorName = sensor;
    if(!robot.hasBodySensor(sensorName))
      mc_rtc::log::error_and_throw("[Observation:{}] Body sensor '{}' does not exist on robot '{}'", obsName,
                                   sensorName, robot.name());

    // FloatingBase is filled by mc_mujoco from the free joint qvel: v in the world frame, ω in the local frame.
    // Any other sensor is an IMU: its gyro gives ω in the sensor frame.
    const std::string defaultSensorFrame = (sensorName == "FloatingBase" && !angular) ? "world" : "local";
    const std::string sensorFrame = parameters("sensor_frame", defaultSensorFrame);
    if(sensorFrame != "world" && sensorFrame != "local")
      mc_rtc::log::error_and_throw("[Observation:{}] sensor_frame must be 'world' or 'local', got '{}'", obsName,
                                   sensorFrame);
    sensorIsWorld = sensorFrame == "world";

    // An IMU does not measure v: refuse instead of silently returning zeros
    if(!angular && sensorName != "FloatingBase" && !parameters.has("sensor_frame"))
      mc_rtc::log::error_and_throw("[Observation:{}] Body sensor '{}' does not provide a linear velocity. Use "
                                   "'body' (observer estimate) or 'sensor: FloatingBase' (simulation), or set "
                                   "'sensor_frame' if your interface fills it",
                                   obsName, sensorName);
  }
  else
  {
    useSensor = false;
    const std::string obs_body = body.empty() ? baseBody : body;
    if(!robot.hasBody(obs_body))
      mc_rtc::log::error_and_throw("[Observation:{}] Body '{}' does not exist on robot '{}'", obsName, obs_body,
                                   robot.name());
    bodyIndex = static_cast<int>(robot.mb().bodyIndexByName(obs_body));
  }

  const std::string frameName = parameters("frame", std::string("local"));
  if(frameName == "local")
    frame = Frame::Local;
  else if(frameName == "world")
    frame = Frame::World;
  else if(frameName == "heading")
    frame = Frame::Heading;
  else
    mc_rtc::log::error_and_throw("[Observation:{}] frame must be 'local', 'world' or 'heading', got '{}'", obsName,
                                 frameName);
}

Eigen::Vector3d VelocitySource::read(const mc_rbdyn::Robot & robot, bool angular) const
{
  // R_0_x: orientation of the source frame in the world frame.
  // sva rotations and BodySensor orientations both store R^T, hence the transposes.
  Eigen::Matrix3d R_0_x;
  Eigen::Vector3d valueLocal;
  if(useSensor)
  {
    const auto & s = robot.bodySensor(sensorName);
    R_0_x = s.orientation().normalized().toRotationMatrix().transpose();
    const Eigen::Vector3d raw = angular ? s.angularVelocity() : s.linearVelocity();
    valueLocal = sensorIsWorld ? Eigen::Vector3d(R_0_x.transpose() * raw) : raw;
  }
  else
  {
    const auto idx = static_cast<size_t>(bodyIndex);
    R_0_x = robot.mbc().bodyPosW[idx].rotation().transpose();
    const auto & velB = robot.mbc().bodyVelB[idx]; // ω and v of the body origin, body frame
    valueLocal = angular ? velB.angular() : velB.linear();
  }

  switch(frame)
  {
    case Frame::World:
      return R_0_x * valueLocal;
    case Frame::Heading:
    {
      const Eigen::Vector3d w = R_0_x * valueLocal;
      const double yaw = std::atan2(R_0_x(1, 0), R_0_x(0, 0));
      const double c = std::cos(yaw);
      const double sn = std::sin(yaw);
      return Eigen::Vector3d(c * w.x() + sn * w.y(), -sn * w.x() + c * w.y(), w.z());
    }
    case Frame::Local:
    default:
      return valueLocal;
  }
}

//============================================================================//
// BaseAngVelObservation
//============================================================================//

BaseAngVelObservation::BaseAngVelObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseAngVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  source_.configure(name(), parameters, observedRobot(context), context.baseBody, true);
  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseAngVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = source_.read(observedRobot(context), true).cwiseProduct(scale_);
}

//============================================================================//
// BaseLinVelObservation
//============================================================================//

BaseLinVelObservation::BaseLinVelObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseLinVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  source_.configure(name(), parameters, observedRobot(context), context.baseBody, false);
  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseLinVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = source_.read(observedRobot(context), false).cwiseProduct(scale_);
}

//============================================================================//
// LastActionObservation
//============================================================================//

LastActionObservation::LastActionObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void LastActionObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  indexes_ = context.convention.resolveJointControllerIndices(parameters, observedRobot(context).refJointOrder(), context.policyJointControllerIndices);
  size_ = static_cast<int>(indexes_.size());
  scale_ = readScale(parameters, "scale", size_, 1.0);
}

void LastActionObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = Eigen::VectorXd::Zero(size_);
  for(size_t i = 0; i < indexes_.size(); ++i)
  {
    const int observedControllerIndex = indexes_[i];

    auto it = std::find(
      context.policyJointControllerIndices.begin(),
      context.policyJointControllerIndices.end(),
      observedControllerIndex);

    if(it == context.policyJointControllerIndices.end())
    {
      continue;
    }

    const Eigen::Index src =
      static_cast<Eigen::Index>(std::distance(context.policyJointControllerIndices.begin(), it));

    if(src >= 0 && src < context.lastActionPolicyOrder.size())
    {
      out(static_cast<Eigen::Index>(i)) =
        context.lastActionPolicyOrder(src) * scale_[i];
    }
  }
}

//============================================================================//
// CommandObservation
//============================================================================//

CommandObservation::CommandObservation(const ObservationConfig & config,
                                       const ObservationConvention & convention)
: Observation(config, convention)
{
}

void CommandObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  size_ = readParameter<int>(parameters, "size", 3);

  if(size_ <= 0 || size_ > 3)
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Command observation supports size in [1, 3], got {}",
      name(),
      size_);
  }

  scale_ = readScale(parameters, "scale", size_, 1.0);
}

void CommandObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  for(int i = 0; i < size_; ++i)
    out(i) = scale_(i) * context.command(i);
}

//============================================================================//
// BaseOrientationObservation
//============================================================================//

BaseOrientationObservation::BaseOrientationObservation(const ObservationConfig & config,
                                                       const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseOrientationObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::string sensor = readParameter<std::string>(parameters, "sensor", std::string(""));
  const std::string body = readParameter<std::string>(parameters, "body", std::string(""));
  if(!sensor.empty() && !body.empty())
    mc_rtc::log::error_and_throw("[Observation:{}] Choose either 'sensor' or 'body' as source, not both", name());

  if(!body.empty())
  {
    useSensor_ = false;
    if(!observedRobot(context).hasBody(body))
      mc_rtc::log::error_and_throw("[Observation:{}] Body '{}' does not exist on robot '{}'", name(), body,
                                   observedRobot(context).name());
    bodyIndex_ = static_cast<int>(observedRobot(context).mb().bodyIndexByName(body));
  }
  else
  {
    useSensor_ = true;
    sensorName_ = sensor.empty() ? std::string("Accelerometer") : sensor;
    if(!observedRobot(context).hasBodySensor(sensorName_))
      mc_rtc::log::error_and_throw("[Observation:{}] Body sensor '{}' does not exist on robot '{}'", name(),
                                   sensorName_, observedRobot(context).name());
  }

  indexes_ = readParameter<std::vector<int>>(parameters, "index", {0,1,2});

  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseOrientationObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  // Both sources store R^T (sva / BodySensor convention), which is what rpyFromMat expects.
  // Normalize the quaternion, not the matrix: Matrix::normalized() divides by the Frobenius norm.
  const Eigen::Matrix3d baseRot =
      useSensor_ ? observedRobot(context).bodySensor(sensorName_).orientation().normalized().toRotationMatrix()
                 : observedRobot(context).mbc().bodyPosW[static_cast<size_t>(bodyIndex_)].rotation();
  const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(baseRot);
  Eigen::VectorXd rpy_scaled = rpy.cwiseProduct(scale_);

  if (indexes_.size() == 3)
    out = rpy_scaled;
  else
    out = rpy_scaled(Eigen::Map<const Eigen::VectorXi>(indexes_.data(), indexes_.size()));
}

//============================================================================//
// PhaseObservation
//============================================================================//

PhaseObservation::PhaseObservation(const ObservationConfig & config,
                                   const ObservationConvention & convention)
: Observation(config, convention)
{
}

void PhaseObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  offset_ = readParameter<double>(parameters, "offset", 0.0);
  scale_ = readScale(parameters, "scale", 2, 1.0);
  cos_first_ = readParameter<bool>(parameters, "cos_first", true);
  commandDeadzone_ = readParameter<double>(parameters, "command_deadzone", 0.0);
}

void PhaseObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  if(commandDeadzone_ > 0.0 && context.command.norm() < commandDeadzone_)
  {
    out.setZero();
    return;
  }
  const double phase = context.phaseNormalized + offset_;
  const double angle = 2.0 * M_PI * phase;
  
  out(1-cos_first_) = scale_(0) * std::cos(angle);
  out(cos_first_) = scale_(1) * std::sin(angle);
}

//============================================================================//
// LogForceSensorObservation
//============================================================================//

LogForceSensorObservation::LogForceSensorObservation(const ObservationConfig & config,
                                                     const ObservationConvention & convention)
: Observation(config, convention)
{
}

void LogForceSensorObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensor_names_ = readParameter<std::vector<std::string>>(parameters, "sensor_names", {});
  if(sensor_names_.size() == 0)
    mc_rtc::log::error_and_throw("[ForceSensorObservation] Please specify at least one force sensor");

  size_ = sensor_names_.size() * 3;
}

void LogForceSensorObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto log1p_compress = [](const Eigen::Vector3d & f) -> Eigen::Vector3d
  {
    return Eigen::Vector3d(std::copysign(std::log1p(std::abs(f.x())), f.x()),
                           std::copysign(std::log1p(std::abs(f.y())), f.y()),
                           std::copysign(std::log1p(std::abs(f.z())), f.z()));
  };
  out = Eigen::VectorXd::Zero(size_);
  int i = 0;
  for(const auto & sensor_name : sensor_names_)
  {
    const auto & forceSensor = observedRobot(context).forceSensor(sensor_name);
    out.segment(i, 3) = log1p_compress(forceSensor.worldWrench(observedRobot(context)).force());
    ;
    i += 3;
  }
}

//============================================================================//
// ForceSensorObservation
//============================================================================//

ForceSensorObservation::ForceSensorObservation(const ObservationConfig & config,
                                               const ObservationConvention & convention)
: Observation(config, convention)
{
}

void ForceSensorObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensor_names_ = readParameter<std::vector<std::string>>(parameters, "sensor_names", {});
  if(sensor_names_.size() == 0)
    mc_rtc::log::error_and_throw("[ForceSensorObservation] Please specify at least one force sensor");

  size_ = sensor_names_.size() * 3;
}

void ForceSensorObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = Eigen::VectorXd::Zero(size_);
  int i = 0;
  for(const auto & sensor_name : sensor_names_)
  {
    const auto & forceSensor = observedRobot(context).forceSensor(sensor_name);
    out.segment(i, 3) = forceSensor.worldWrench(observedRobot(context)).force();
    ;
    i += 3;
  }
}

} // namespace rlqp
