#pragma once

#include <Eigen/Core>

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rlqp
{

/**
 * @brief Training-environment convention defaults.
 * ex: mjlab exposes joint_pos_rel and generated_commands (names in mjlab),
 * implemented internally by joint_pos and command with a default config.
 */
struct ObservationConvention
{
  /** @brief Convention name, e.g. "mjlab" or "isaaclab". */
  std::string name = "mjlab";

  /** @brief Named joint groups, e.g. all, legs, arms. */
  std::map<std::string, std::vector<std::string> > jointGroups;

  /**
   * @brief Default parameters keyed by exact external observation name.
   * ex: mjlab::joint_pos_rel can have defaults that differ from a generic
   * joint_pos implementation.
   */
  std::map<std::string, mc_rtc::Configuration> defaultParameters;

  /** @brief Maps external observation names to internal generic implementations. */
  std::map<std::string, std::string> typeAliases;

  /** @brief Load one convention block from a file with a "conventions" entry */
  static ObservationConvention fromConfig(const mc_rtc::Configuration & controllerConfig,
                                          const std::string & conventionName);

  /** @brief Resolve an external YAML type to an internal registered observation type. */
  std::string resolveType(const std::string & requestedType) const;

  /** @brief Resolve an observation joint selector to controller joint indices. */
  std::vector<int> resolveJointControllerIndices(const mc_rtc::Configuration & parameters,
                                                 const std::vector<std::string> & controllerJointOrder,
                                                 const std::vector<int> & fallbackIndices) const;

  /** @brief Resolve final parameters for one observation. */
  mc_rtc::Configuration resolveObservationParameters(const std::string & requestedType,
                                                     const std::string & internalType,
                                                     const mc_rtc::Configuration & localParameters) const;
};

/**
 * @brief Runtime data passed to observations during configure() and compute().
 *
 * Both robots are provided; each observation reads the one selected by its
 * observation_source (see ObservationConfig) through Observation::observedRobot().
 *
 * Joint-order convention:
 * - controllerJointOrder is mc_rtc's robot().refJointOrder().
 * - policyJointControllerIndices maps action index i to controllerJointOrder index,
 * - qZeroControllerOrder is stored in mc_rtc join order.
 * - lastActionPolicyOrder is stored in RL joint order.
 */
struct ObservationContext
{
  /** @brief Measured robot state: ctl.realRobot(...). */
  mc_rbdyn::Robot & realRobot;

  /** @brief Control robot state: ctl.robot(). */
  mc_rbdyn::Robot & controlRobot;

  /** @brief Robot base body used by base velocity and projected gravity observations. */
  std::string baseBody;

  /** @brief mc_rtc controller order: robot().refJointOrder(). Used at configure time only. */
  const std::vector<std::string> & controllerJointOrder;

  /** @brief Maps policy joints as controller indices in RL convention order */
  const std::vector<int> & policyJointControllerIndices;

  /** @brief Default pose expanded in controllerJointOrder. */
  const Eigen::VectorXd & qZeroControllerOrder;

  /** @brief Last raw policy action in policyJointOrder. */
  const Eigen::VectorXd & lastActionPolicyOrder;

  /** @brief High-level command vector, e.g. vx, vy, yaw_rate. */
  const Eigen::Vector3d & command;

  /**
   * @brief Normalized control phase in [0, 1).
   *
   * PhaseObservation converts this scalar into [cos(2*pi*phase), sin(2*pi*phase)].
   * The phase value is updated by RLPolicyRuntime.
   */
  double phaseNormalized = 0.0;

  /** @brief Active training-environment convention. */
  ObservationConvention convention;
};

/** @brief Parsed declaration of one observation entry from observations.yaml. */
struct ObservationConfig
{
  /** @brief Internal implementation type after convention alias resolution. */
  std::string type;

  /** @brief Exact type requested in YAML, e.g. joint_pos_rel. */
  std::string requestedType;

  /**
   * @brief Optional user/debug label.
   *
   * Useful when the same observation type appears multiple times with
   * different parameters. If absent, it defaults to requestedType.
   */
  std::string name;

  /**
   * @brief Number of samples stored by ObservationManager for this observation.
   *
   * Resolved by ObservationManager: parameters.history of the entry, else the
   * top-level history of observations.yaml, else 1.
   */
  int history = 1;

  /**
   * @brief Robot the observation reads: realRobot (measured state) or robot (control state).
   *
   * Resolved by ObservationManager: parameters.observation_source of the entry, else the
   * top-level observation_source of observations.yaml, else realRobot.
   */
  bool useRealRobot = true;

  /** @brief Observation-specific parameter block. */
  mc_rtc::Configuration parameters;
};

/**
 * @brief Base class for one observation term.
 *
 * Subclasses own observation-specific state such as resolved joint indices,
 * body index, default pose and scale.
 */
class Observation
{
public:
  Observation(const ObservationConfig & config, const ObservationConvention & convention);
  virtual ~Observation();

  const std::string & type() const { return config_.type; }
  const std::string & requestedType() const { return config_.requestedType; }
  const std::string & name() const { return config_.name; }
  int history() const { return config_.history; }
  const char * observationSource() const { return config_.useRealRobot ? "realRobot" : "robot"; }

  /** @brief Resolve configuration-time data into runtime indices/state. */
  virtual void configure(const ObservationContext & context) = 0;

  /** @brief Size of one sample of this observation, excluding history. */
  virtual int size() const = 0;

  /** @brief Compute one sample into out. */
  virtual void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const = 0;

protected:
  /** @brief Robot selected by this observation's observation_source. */
  mc_rbdyn::Robot & observedRobot(const ObservationContext & context) const
  {
    return config_.useRealRobot ? context.realRobot : context.controlRobot;
  }

  /** @brief Read a typed parameter with a fallback value. */
  template<typename T>
  T readParameter(const mc_rtc::Configuration & parameters,
                  const std::string & key,
                  const T & fallback) const
  {
    if(!parameters.has(key))
      return fallback;
    T value = fallback;
    parameters(key, value);
    return value;
  }

  /** @brief Read a scalar or vector scale parameter. */
  Eigen::VectorXd readScale(const mc_rtc::Configuration & parameters,
                                  const std::string & key,
                                  int size,
                                  double fallback) const;

protected:
  ObservationConfig config_;
  ObservationConvention convention_;
};

/** @brief Registry linking known observations to the appropriate class */
class ObservationRegistry
{
public:
  using ObservationFactory =
    std::function<std::shared_ptr<Observation>(const ObservationConfig &, const ObservationConvention &)>;

  void registerType(const std::string & type, ObservationFactory factory);
  std::shared_ptr<Observation> create(const ObservationConfig & config,
                                      const ObservationConvention & convention) const;

  std::vector<std::string> knownTypes() const;

private:
  std::map<std::string, ObservationFactory> factories_;
};

ObservationRegistry makeDefaultObservationRegistry();

} // namespace rlqp
