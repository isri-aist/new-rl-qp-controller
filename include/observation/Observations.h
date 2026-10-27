#pragma once

#include "Observation.h"

#include <Eigen/Core>

#include <mc_rtc/Configuration.h>

#include <string>
#include <vector>

namespace rlqp
{

/** @brief Joint position observation, optionally relative to q_zero. */
class JointPosObservation : public Observation
{
public:
  JointPosObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return static_cast<int>(controllerIndices_.size());
  }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<int> controllerIndices_;
  std::vector<int> mbcIndices_;
  bool relativeToDefaultPose_ = true;
  Eigen::VectorXd defaultPose_;
  Eigen::VectorXd scale_;
};

/** @brief Joint velocity observation, optionally relative to a default velocity. */
class JointVelObservation : public Observation
{
public:
  JointVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return static_cast<int>(controllerIndices_.size());
  }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<int> controllerIndices_;
  std::vector<int> mbcIndices_;
  bool relativeToDefaultVelocity_ = true;
  Eigen::VectorXd defaultVelocity_;
  Eigen::VectorXd scale_;
};

/** @brief Projected gravity in the configured base body frame. */
class ProjectedGravityObservation : public Observation
{
public:
  ProjectedGravityObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  int bodyIndex_ = -1;
  Eigen::VectorXd scale_;
  bool useSensor_ = false;
  std::string sensorName_ = "FloatingBase";
};

/**
 * @brief Where a velocity observation reads ω or v, and in which frame it outputs it.
 *
 * Used by base_ang_vel and base_lin_vel; each observation entry has its own source and frame.
 *
 * Source (pick one, default: body = robot.base_body):
 *   body: <body name>          ω and v of the body origin from the observed robot state (any observer)
 *   sensor: <body sensor name> ω (or v) read from a BodySensor
 *
 * sensor_frame: world | local   frame the sensor stores its velocity in
 *                               default: for "FloatingBase" (mc_mujoco free joint qvel), world for v and local for ω,
 *                                        local for any other sensor (a gyro measures ω in its own frame)
 *
 * frame: local | world | heading  output frame (default: local)
 *   local   : frame of the body (or of the sensor)
 *   world   : world frame
 *   heading : world frame rotated by the source yaw only (yaw-aligned, gravity stays on z)
 */
struct VelocitySource
{
  enum class Frame
  {
    Local,
    World,
    Heading
  };

  bool useSensor = false;
  bool sensorIsWorld = false;
  std::string sensorName;
  int bodyIndex = -1;
  Frame frame = Frame::Local;

  void configure(const std::string & obsName,
                 const mc_rtc::Configuration & parameters,
                 const ObservationContext & context,
                 bool angular);
  Eigen::Vector3d read(const ObservationContext & context, bool angular) const;
};

/** @brief Angular velocity ω of the base, or of any body (see VelocitySource for parameters). */
class BaseAngVelObservation : public Observation
{
public:
  BaseAngVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  VelocitySource source_;
  Eigen::VectorXd scale_;
};

/** @brief Linear velocity v of the base origin, or of any body (see VelocitySource for parameters). */
class BaseLinVelObservation : public Observation
{
public:
  BaseLinVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  VelocitySource source_;
  Eigen::VectorXd scale_;
};

/** @brief Last raw policy action. */
class LastActionObservation : public Observation
{
public:
  LastActionObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return size_; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;
  Eigen::VectorXd makeFullLastActionPolicyOrder(
    const Eigen::VectorXd & rawAction,
    const std::vector<int> & actionJointIndicesPolicyOrder,
    int fullPolicyActionSize,
    double actionScale);

private:
  int size_ = 0;
  std::vector<int> indexes_ = {};
  Eigen::VectorXd scale_;
};

/** @brief Command observation, usually vx/vy/yaw_rate. */
class CommandObservation : public Observation
{
public:
  CommandObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return size_; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  int size_ = 3;
  Eigen::VectorXd scale_;
};


/**
 * @brief Base roll/pitch/yaw observation.
 *
 * Source (pick one, default: sensor = Accelerometer):
 *   sensor: <body sensor name>  orientation of a BodySensor (filled by the simulator, the IMU or an observer)
 *   body: <body name>           orientation from the observed robot state (any floating-base observer)
 */
class BaseOrientationObservation : public Observation
{
public:
  BaseOrientationObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return indexes_.size(); }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  bool useSensor_ = true;
  std::string sensorName_ = "Accelerometer";
  int bodyIndex_ = -1;
  std::vector<int> indexes_ = {0,1,2};
  Eigen::VectorXd scale_;
};

/**
 * @brief Phase observation represented as [cos(phase), sin(phase)].
 *
 * The runtime owns the normalized phase in [0, 1). This observation only
 * converts it to the trigonometric representation expected by many locomotion
 * policies.
 */
class PhaseObservation : public Observation
{
public:
  PhaseObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 2; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  double offset_ = 0.0;
  Eigen::VectorXd scale_;
  bool cos_first_ = true;
  double commandDeadzone_ = 0.0;
};

/**
 * @brief Observation from a force sensor by name - log compressed data
 *
 * Handles an array of sensor, their data will be concatenated
 */
class LogForceSensorObservation : public Observation
{
public:
  LogForceSensorObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return size_;
  };
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<std::string> sensor_names_ = {};
  int size_ = 0;
};

/**
 * @brief Observation from a force sensor by name - raw data
 *
 * Handles an array of sensor, their data will be concatenated
 */
class ForceSensorObservation : public Observation
{
public:
  ForceSensorObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return size_;
  };
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<std::string> sensor_names_ = {};
  int size_ = 0;
};

} // namespace rlqp
