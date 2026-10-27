#include "policy/PolicyConfig.h"
#include "policy/RLPolicyInterface.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <fstream>
#include <limits>
#include <sstream>
#include <sys/stat.h>

namespace rlqp
{
//============================================================================//
// Helper function
//============================================================================//
namespace
{
/** Split a comma-separated metadata value, trimming spaces */
std::vector<std::string> splitList(const std::string & value)
{
  std::vector<std::string> out;
  std::stringstream ss(value);
  std::string item;
  while(std::getline(ss, item, ','))
  {
    const size_t first = item.find_first_not_of(" \t\n");
    const size_t last = item.find_last_not_of(" \t\n");
    if(first != std::string::npos) out.push_back(item.substr(first, last - first + 1));
  }
  return out;
}

/** Check if same values up to rounding*/
bool sameValue(double yaml, double metadata)
{
  return std::abs(yaml - metadata) <= 1e-3 + 1e-2 * std::abs(metadata);
}

/** Read a per-joint map from policy.yaml; every key must be a controller joint (catches typos) */
std::map<std::string, double> readJointMap(const mc_rtc::Configuration & config,
                                           const std::string & key,
                                           const std::vector<std::string> & joints,
                                           const std::string & policyName)
{
  const std::map<std::string, double> out = config(key, std::map<std::string, double>());
  for(const auto & entry : out)
  {
    if(std::find(joints.begin(), joints.end(), entry.first) == joints.end())
      mc_rtc::log::error_and_throw("[PolicyConfig:{}] policy.yaml: '{}' has an entry for '{}', which is not a joint of "
                                   "the controller robot",
                                   policyName, key, entry.first);
  }
  return out;
}

/**
 * Resolve one per-joint value from policy.yaml and the ONNX metadata.
 *
 * yamlWins: deployment choices (kp, kd) keep the policy.yaml value; training constants (q0, action scale)
 * keep the metadata value, since the network was trained with it. Differences beyond rounding are collected.
 */
bool resolveJointValue(const std::string & joint,
                       const std::map<std::string, double> & yaml,
                       const std::map<std::string, double> & metadata,
                       bool yamlWins,
                       double & value,
                       std::vector<std::string> & differences)
{
  const auto y = yaml.find(joint);
  const auto m = metadata.find(joint);
  if(y != yaml.end() && m != metadata.end())
  {
    if(!sameValue(y->second, m->second))
      differences.push_back(fmt::format("{} (policy.yaml {}, ONNX {})", joint, y->second, m->second));
    value = yamlWins ? y->second : m->second;
    return true;
  }
  if(y != yaml.end()) value = y->second;
  else if(m != metadata.end()) value = m->second;
  return y != yaml.end() || m != metadata.end();
}

void warnDifferences(const std::string & policyName,
                     const std::string & field,
                     const std::string & used,
                     const std::vector<std::string> & differences)
{
  if(differences.empty()) return;
  std::string list;
  for(const auto & d : differences) list += "\n  " + d;
  mc_rtc::log::warning("[PolicyConfig:{}] {} differs from the ONNX metadata, using the {} value for:{}", policyName,
                       field, used, list);
}

std::string basenameWithoutExtension(const std::string & path)
{
  std::string base = path;

  const size_t slash = base.find_last_of('/');
  if(slash != std::string::npos) base = base.substr(slash + 1);

  const size_t dot = base.find_last_of('.');
  if(dot != std::string::npos) base = base.substr(0, dot);

  return base;
}

std::string joinPath(const std::string & lhs, const std::string & rhs)
{
  if(lhs.empty()) return rhs;

  if(lhs.back() == '/') return lhs + rhs;

  return lhs + "/" + rhs;
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path.c_str());
  return file.good();
}

bool hasOnnxFile(const std::string & folder)
{
  DIR * dir = opendir(folder.c_str());
  if(!dir) return false;

  bool found = false;
  struct dirent * entry = nullptr;
  while((entry = readdir(dir)) != nullptr)
  {
    const std::string name = entry->d_name;
    if(name.size() >= 5 && name.substr(name.size() - 5) == ".onnx")
    {
      found = true;
      break;
    }
  }

  closedir(dir);
  return found;
}

bool isPolicyFolder(const std::string & folder)
{
  return fileExists(joinPath(folder, "policy.yaml")) && fileExists(joinPath(folder, "observations.yaml"))
         && hasOnnxFile(folder);
}

std::vector<std::string> listPolicies(const std::string & root, const std::vector<std::string> & requiredFiles)
{
  std::vector<std::string> folders;

  DIR * dir = opendir(root.c_str());
  if(!dir) mc_rtc::log::error_and_throw("[PolicyConfig] Could not open directory '{}'", root);

  struct dirent * entry = nullptr;
  while((entry = readdir(dir)) != nullptr)
  {
    const std::string name = entry->d_name;
    if(name == "." || name == "..") continue;

    const std::string folder = joinPath(root, name);
    struct stat status;
    if(stat(folder.c_str(), &status) != 0 || !S_ISDIR(status.st_mode)) continue;

    bool complete = true;
    for(size_t i = 0; i < requiredFiles.size(); ++i)
    {
      std::ifstream file(joinPath(folder, requiredFiles[i]).c_str());
      if(!file.good())
      {
        complete = false;
        break;
      }
    }

    if(complete) folders.push_back(folder);
  }

  closedir(dir);
  std::sort(folders.begin(), folders.end());
  return folders;
}
} // namespace

//============================================================================//
// OnnxMetadata
//============================================================================//
OnnxMetadata OnnxMetadata::parse(const std::map<std::string, std::string> & raw, const std::string & policyName)
{
  OnnxMetadata out;

  auto find = [&raw](const std::string & key) -> const std::string *
  {
    const auto it = raw.find(key);
    return it == raw.end() ? nullptr : &it->second;
  };

  if(const std::string * names = find("joint_names")) out.jointNames = splitList(*names);
  if(const std::string * names = find("observation_names")) out.observationNames = splitList(*names);

  // Per-joint values are stored in joint_names order
  auto parseJointValues = [&](const std::string & key, std::map<std::string, double> & target)
  {
    const std::string * value = find(key);
    if(!value) return;
    if(out.jointNames.empty())
    {
      mc_rtc::log::warning("[PolicyConfig:{}] ONNX metadata '{}' ignored: no joint_names to attach it to", policyName,
                           key);
      return;
    }
    const std::vector<std::string> items = splitList(*value);
    if(items.size() != out.jointNames.size())
    {
      mc_rtc::log::warning("[PolicyConfig:{}] ONNX metadata '{}' ignored: {} values for {} joint_names", policyName,
                           key, items.size(), out.jointNames.size());
      return;
    }
    try
    {
      for(size_t i = 0; i < items.size(); ++i) target[out.jointNames[i]] = std::stod(items[i]);
    }
    catch(const std::exception &)
    {
      target.clear();
      mc_rtc::log::warning("[PolicyConfig:{}] ONNX metadata '{}' ignored: not a list of numbers", policyName, key);
    }
  };

  parseJointValues("joint_stiffness", out.kp);
  parseJointValues("joint_damping", out.kd);
  parseJointValues("default_joint_pos", out.q0);
  parseJointValues("action_scale", out.actionScale);

  return out;
}

//============================================================================//
// PolicyConfig
//============================================================================//
PolicyConfig PolicyConfig::load(const std::string & policyFolder,
                                const std::vector<std::string> & mcRtcJoints,
                                const mc_rtc::Configuration & controllerConfig)
{
  PolicyConfig out;

  out.folder = policyFolder;
  out.policyYamlPath = joinPath(policyFolder, "policy.yaml");
  out.observationsYamlPath = joinPath(policyFolder, "observations.yaml");

  out.policyConfiguration.load(out.policyYamlPath);
  out.observationsConfiguration.load(out.observationsYamlPath);

  const size_t joint_size = mcRtcJoints.size();

  out.name = out.policyConfiguration("name", basenameWithoutExtension(policyFolder));

  const std::string defaultOnnxName = out.name + ".onnx";
  const std::string onnxFile = out.policyConfiguration("onnx", defaultOnnxName);
  out.onnxPath = joinPath(policyFolder, onnxFile);

  // ONNX metadata: the training data the exporter stored, if any
  if(out.policyConfiguration("use_onnx_metadata", true))
  {
    // An unreadable file only fails when the policy is selected, as before: no metadata here
    try
    {
      out.metadata = OnnxMetadata::parse(RLPolicyInterface::readMetadata(out.onnxPath), out.name);
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::warning("[PolicyConfig:{}] Could not read the metadata of '{}': {}", out.name, out.onnxPath,
                           e.what());
    }
    for(const auto & joint : out.metadata.jointNames)
    {
      if(std::find(mcRtcJoints.begin(), mcRtcJoints.end(), joint) == mcRtcJoints.end())
        mc_rtc::log::error_and_throw("[PolicyConfig:{}] ONNX metadata joint '{}' is not a joint of the controller "
                                     "robot: wrong robot or wrong ONNX file",
                                     out.name, joint);
    }
  }

  // Training convention (optional), with the metadata joint order as its "all" group
  out.convention = ObservationConvention::fromConfig(
      controllerConfig, out.observationsConfiguration("training_convention", std::string("")));
  if(!out.metadata.jointNames.empty())
  {
    const auto all = out.convention.jointGroups.find("all");
    if(all != out.convention.jointGroups.end() && all->second != out.metadata.jointNames)
      mc_rtc::log::error_and_throw("[PolicyConfig:{}] Joint group 'all' of convention '{}' does not match the ONNX "
                                   "metadata joint order.\n  convention: {}\n  ONNX:       {}\nRemove the group to use "
                                   "the ONNX order, or set use_onnx_metadata: false in policy.yaml",
                                   out.name, out.convention.name, all->second, out.metadata.jointNames);
    out.convention.jointGroups["all"] = out.metadata.jointNames;
  }

  std::map<std::string, double> kp_map, kd_map, defaultPos_map, actionScale_map;
  double actionScaleAll = 1.0;
  bool yamlScalarScale = false;

  if(out.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = out.policyConfiguration("control");

    out.useQP = control("use_QP", out.policyConfiguration("use_QP", true));

    const bool hasPeriod = control.has("period_s");
    const bool hasFrequency = control.has("frequency_hz");
    const bool hasCmdVelSpeed = control.has("vel_cmd_speed");
    if(hasPeriod && hasFrequency)
      mc_rtc::log::error_and_throw("[PolicyConfig:{}] Specify only one of control.period_s or control.frequency_hz",
                                   out.name);

    if(hasPeriod)
      out.policyStepSize = control("period_s", 0.02);
    else if(hasFrequency)
    {
      const double frequency = control("frequency_hz", 50.0);
      if(frequency <= 0.0)
        mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.frequency_hz must be positive", out.name);
      out.policyStepSize = 1.0 / frequency;
    }
    out.pdGainsRatio = control("pd_gains_ratio", 1.0);

    if(hasCmdVelSpeed)
    {
      VelocityCommand vel_cmd{control("vel_cmd_speed")("xy", 0.4), control("vel_cmd_speed")("yaw", 0.4)};
      out.velCmd = vel_cmd;
    }

    kp_map = readJointMap(control, "kp", mcRtcJoints, out.name);
    kd_map = readJointMap(control, "kd", mcRtcJoints, out.name);
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"control\" entry");
  if(out.policyConfiguration.has("action"))
  {
    const mc_rtc::Configuration action = out.policyConfiguration("action");
    out.actionJointGroup = "all";
    if(action.has("joints"))
    {
      try
      {
        out.actionJointGroup = action("joints").operator std::string();
      }
      catch(mc_rtc::Configuration::Exception & e)
      {
        e.silence();
        mc_rtc::log::error_and_throw("[PolicyConfig:{}] action.joints must be the name of a joint group", out.name);
      }
    }
    out.controlledJointGroup = out.actionJointGroup;
    if(action.has("controlled_joints")) action("controlled_joints", out.controlledJointGroup);

    actionScale_map = readJointMap(action, "scale", mcRtcJoints, out.name);
    yamlScalarScale = actionScale_map.empty() && action.has("scale");
    if(yamlScalarScale) actionScaleAll = action("scale", 1.0);
    defaultPos_map = readJointMap(action, "q0", mcRtcJoints, out.name);
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"action\" entry");

  out.kp = std::vector<double>(joint_size);
  out.kd = std::vector<double>(joint_size);
  out.actionScale = std::vector<double>(joint_size, actionScaleAll);
  out.defaultPosition = std::vector<double>(joint_size, std::numeric_limits<double>::quiet_NaN());

  std::vector<std::string> missingKp, missingKd;
  std::vector<std::string> kpDiff, kdDiff, q0Diff, scaleDiff;
  for(size_t i = 0; i < joint_size; ++i)
  {
    const std::string & joint = mcRtcJoints[i];

    if(!resolveJointValue(joint, kp_map, out.metadata.kp, true, out.kp[i], kpDiff)) missingKp.push_back(joint);
    if(!resolveJointValue(joint, kd_map, out.metadata.kd, true, out.kd[i], kdDiff)) missingKd.push_back(joint);
    resolveJointValue(joint, defaultPos_map, out.metadata.q0, false, out.defaultPosition[i], q0Diff);
    resolveJointValue(joint, actionScale_map, out.metadata.actionScale, false, out.actionScale[i], scaleDiff);

    // A scalar action.scale is a policy.yaml value too: compare it with the metadata
    const auto m = out.metadata.actionScale.find(joint);
    if(yamlScalarScale && m != out.metadata.actionScale.end() && !sameValue(actionScaleAll, m->second))
      scaleDiff.push_back(fmt::format("{} (policy.yaml {}, ONNX {})", joint, actionScaleAll, m->second));
  }

  if(!missingKp.empty() || !missingKd.empty())
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] No gain in policy.yaml nor in the ONNX metadata for:\n  kp: {}\n  "
                                 "kd: {}",
                                 out.name, missingKp, missingKd);

  warnDifferences(out.name, "control.kp", "policy.yaml", kpDiff);
  warnDifferences(out.name, "control.kd", "policy.yaml", kdDiff);
  warnDifferences(out.name, "action.q0", "ONNX", q0Diff);
  warnDifferences(out.name, "action.scale", "ONNX", scaleDiff);

  if(!out.metadata.jointNames.empty())
    mc_rtc::log::info("[PolicyConfig:{}] ONNX metadata: {} joints{}{}{}{}{}", out.name,
                      out.metadata.jointNames.size(), out.metadata.kp.empty() ? "" : ", kp",
                      out.metadata.kd.empty() ? "" : ", kd", out.metadata.q0.empty() ? "" : ", q0",
                      out.metadata.actionScale.empty() ? "" : ", action scale",
                      out.metadata.observationNames.empty() ? "" : ", observation names");

  if(out.controlledJointGroup.empty()) out.controlledJointGroup = out.actionJointGroup;

  out.validate();
  return out;
}

void PolicyConfig::validate() const
{
  if(name.empty()) mc_rtc::log::error_and_throw("[PolicyConfig] Policy name cannot be empty");

  if(folder.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] Policy folder cannot be empty", name);

  if(onnxPath.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] ONNX path cannot be empty", name);

  if(actionJointGroup.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] action.joints cannot be empty", name);

  if(policyStepSize <= 0.0) mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.period_s must be positive", name);

  if(pdGainsRatio <= 0.0)
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.pd_gains_ratio must be positive", name);
}

//============================================================================//
// PolicyManager
//============================================================================//
void PolicyManager::load(const mc_rtc::Configuration & controllerConfig, std::vector<std::string> mcRtcJoints)
{
  orderedNames_.clear();
  policies_.clear();
  currentName_.clear();

  const std::string policiesRoot = controllerConfig("policies_root", std::string("policies"));
  const bool runCurrentFolderPolicy = isPolicyFolder(".");
  std::vector<std::string> folders = listPolicies(policiesRoot, {"policy.yaml", "observations.yaml"});
  if(runCurrentFolderPolicy) folders.insert(folders.begin(), ".");

  if(folders.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyManager] No policy folders found in '{}'. Expected subdirectories containing "
                                 "policy.yaml and observations.yaml.",
                                 policiesRoot);
  }

  for(size_t i = 0; i < folders.size(); ++i)
  {
    PolicyConfig policy = PolicyConfig::load(folders[i], mcRtcJoints, controllerConfig);

    if(policies_.find(policy.name) != policies_.end())
    {
      if(runCurrentFolderPolicy) continue;
      mc_rtc::log::error_and_throw("[PolicyManager] Duplicate policy name '{}'", policy.name);
    }

    orderedNames_.push_back(policy.name);
    policies_[policy.name] = policy;
  }

  std::string defaultPolicy = orderedNames_.front();
  if(!runCurrentFolderPolicy) controllerConfig("default_policy", defaultPolicy);
  select(defaultPolicy);

  mc_rtc::log::success("[PolicyManager] Loaded {} policy{} from '{}'. Active policy: {}", policies_.size(),
                       policies_.size() == 1 ? "" : "s",
                       runCurrentFolderPolicy ? std::string("current directory") : policiesRoot, currentName_);
}

const PolicyConfig & PolicyManager::get(const std::string & name) const
{
  std::map<std::string, PolicyConfig>::const_iterator it = policies_.find(name);

  if(it == policies_.end()) mc_rtc::log::error_and_throw("[PolicyManager] Unknown policy '{}'", name);

  return it->second;
}

void PolicyManager::select(const std::string & name)
{
  get(name);
  currentName_ = name;
}

void PolicyManager::selectNext()
{
  if(orderedNames_.empty())
    mc_rtc::log::error_and_throw("[PolicyManager] Cannot select next policy: no policies loaded");

  std::vector<std::string>::const_iterator it = std::find(orderedNames_.begin(), orderedNames_.end(), currentName_);

  if(it == orderedNames_.end())
  {
    currentName_ = orderedNames_.front();
    return;
  }

  ++it;

  if(it == orderedNames_.end()) it = orderedNames_.begin();

  currentName_ = *it;
}

} // namespace rlqp
