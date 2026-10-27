#pragma once

#include "observation/Observation.h"

#include <mc_rtc/Configuration.h>

#include <map>
#include <string>
#include <vector>

namespace rlqp
{
/**
 * @brief Stores a policy's velocity commands
 */
struct VelocityCommand
{
  double xy = 0.4;
  double yaw = 0.4;
};

/**
 * @brief Training data an exporter may store in the ONNX custom metadata (mjlab keys).
 *
 * Every field is optional: an ONNX file may carry none of them. Per-joint values are keyed by joint name.
 */
struct OnnxMetadata
{
  /** @brief joint_names: policy joint order, used as the implicit "all" joint group */
  std::vector<std::string> jointNames;
  /** @brief joint_stiffness, joint_damping, default_joint_pos, action_scale */
  std::map<std::string, double> kp;
  std::map<std::string, double> kd;
  std::map<std::string, double> q0;
  std::map<std::string, double> actionScale;
  /** @brief observation_names: observation terms, in input order */
  std::vector<std::string> observationNames;

  /** @brief Parse the raw metadata map; malformed or inconsistent keys are skipped with a warning */
  static OnnxMetadata parse(const std::map<std::string, std::string> & raw, const std::string & policyName);
};

/**
 * @brief Owns a policy configuration and informations
 * All data is stored in mc_rtc joint order.
 */
struct PolicyConfig
{
  std::string name;
  std::string folder;
  std::string onnxPath;
  std::string policyYamlPath;
  std::string observationsYamlPath;
  VelocityCommand velCmd;

  bool useQP = true;
  /** Policy inference period in seconds. Configured through control.period_s or control.frequency_hz. */
  double policyStepSize = 0.02;
  /** Runtime gain ratio: Kp = ratio * kp_base, Kd = sqrt(ratio) * kd_base. */
  double pdGainsRatio = 1.0;

  /** @brief Joint-group selector from policy.yaml/action/joints. This describes the ONNX action vector layout/size. */
  std::string actionJointGroup;

  /**
   * @brief Joint-group selector from policy.yaml/action/controlled_joints.
   *
   * This describes which of the ONNX action outputs are actually applied to q_rl.
   * If omitted, it defaults to actionJointGroup for backward compatibility.
   */
  std::string controlledJointGroup;

  /** @brief Optional per-joint action scale overrides. Missing entries will use scale 1.0. */
  std::vector<double> actionScale;

  /**
   * @brief Default target pose, one entry per controller joint.
   *
   * NaN where neither the ONNX metadata nor policy.yaml gives a value: the runtime then uses the posture task.
   */
  std::vector<double> defaultPosition;

  /** @brief Mandatory kp and kd */
  std::vector<double> kp;
  std::vector<double> kd;

  /** @brief ONNX metadata, empty if the file has none or policy.yaml sets use_onnx_metadata: false */
  OnnxMetadata metadata;

  /** @brief Training convention of observations.yaml, with the metadata joint order as its "all" group */
  ObservationConvention convention;

  mc_rtc::Configuration policyConfiguration;
  mc_rtc::Configuration observationsConfiguration;

  static PolicyConfig load(const std::string & policyFolder,
                           const std::vector<std::string> & mcRtcJoints,
                           const mc_rtc::Configuration & controllerConfig);

  void validate() const;
};

/** @brief Handles the loadable policies */
class PolicyManager
{
public:
  void load(const mc_rtc::Configuration & controllerConfig, std::vector<std::string> mcRtcJoints);

  bool empty() const
  {
    return policies_.empty();
  };

  size_t size() const
  {
    return policies_.size();
  };

  const PolicyConfig & current() const
  {
    return get(currentName_);
  };

  const std::string & currentName() const
  {
    return currentName_;
  };

  const PolicyConfig & get(const std::string & name) const;

  const std::vector<std::string> & names() const
  {
    return orderedNames_;
  };

  void select(const std::string & name);
  void selectNext();

private:
  std::vector<std::string> orderedNames_;
  std::map<std::string, PolicyConfig> policies_;
  std::string currentName_;
};

} // namespace rlqp
