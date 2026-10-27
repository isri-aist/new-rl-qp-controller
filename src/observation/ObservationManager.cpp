#include "observation/ObservationManager.h"

#include <mc_rtc/logging.h>

namespace rlqp
{

namespace
{

mc_rtc::Configuration emptyConfiguration()
{
  return mc_rtc::Configuration();
}

bool parseObservationSource(const std::string & source, const std::string & where)
{
  if(source != "realRobot" && source != "robot")
    mc_rtc::log::error_and_throw("[ObservationManager] Invalid observation_source '{}' in {}. Expected 'realRobot' or 'robot'",
                                 source, where);
  return source == "realRobot";
}

} // namespace

ObservationManager::ObservationManager() {}

void ObservationManager::load(const mc_rtc::Configuration & observationsConfig,
                              const mc_rtc::Configuration & controllerConfig,
                              const ObservationRegistry & registry)
{
  entries_.clear();
  size_ = 0;

  const std::string conventionName = observationsConfig("training_convention", std::string("mjlab"));
  convention_ = ObservationConvention::fromConfig(controllerConfig, conventionName);

  if(!observationsConfig.has("observations"))
    mc_rtc::log::error_and_throw("[ObservationManager] observations.yaml has no required 'observations' list");

  mc_rtc::Configuration observations = observationsConfig("observations");

  // Policy-wide defaults, overridden by each entry's parameters
  ObservationConfig defaults;
  defaults.history = observationsConfig("history", 1);
  if(defaults.history <= 0)
    mc_rtc::log::error_and_throw("[ObservationManager] observations.yaml has invalid history {}", defaults.history);
  defaultSource_ = observationsConfig("observation_source", std::string("realRobot"));
  defaults.useRealRobot = parseObservationSource(defaultSource_, "observations.yaml");

  Entry entry;
  for(size_t i = 0; i < observations.size(); ++i)
  {
    ObservationConfig config = parseObservationConfig(observations[i], defaults);
    config.type = convention_.resolveType(config.requestedType);

    entry.observation = registry.create(config, convention_);
    entries_.push_back(entry);
  }

  newest_first_ = observationsConfig("history_order_newest_first", true);

  if(entries_.empty())
    mc_rtc::log::error_and_throw("[ObservationManager] observations.yaml defines an empty observation list");

  mc_rtc::log::success("[ObservationManager] Loaded {} observations with '{}' convention", entries_.size(),
                       convention_.name);
}

void ObservationManager::configure(const ObservationContext & context)
{
  size_ = 0;

  for(size_t i = 0; i < entries_.size(); ++i)
  {
    entries_[i].observation->configure(context);
    size_ += entries_[i].observation->history() * entries_[i].observation->size();
  }
}

void ObservationManager::updateHistory(const ObservationContext & context)
{
  for(size_t i = 0; i < entries_.size(); ++i)
  {
    Entry & entry = entries_[i];
    entry.historyBuffer.clear();

    Eigen::VectorXd current = Eigen::VectorXd::Zero(entry.observation->size());
    entry.observation->compute(context, current);

    for(int h = 0; h < entry.observation->history(); ++h) entry.historyBuffer.push_front(current);
  }
}

Eigen::VectorXd ObservationManager::compute(const ObservationContext & context)
{
  Eigen::VectorXd out = Eigen::VectorXd::Zero(size_);
  int offset = 0;

  for(size_t i = 0; i < entries_.size(); ++i)
  {
    Entry & entry = entries_[i];

    Eigen::VectorXd current = Eigen::VectorXd::Zero(entry.observation->size());
    entry.observation->compute(context, current);

    entry.historyBuffer.push_front(current);
    while(static_cast<int>(entry.historyBuffer.size()) > entry.observation->history()) entry.historyBuffer.pop_back();

    Eigen::VectorXd flatten = flattenHistory(entry);
    out.segment(offset, flatten.size()) = flatten;
    offset += static_cast<int>(flatten.size());
  }

  return out;
}

ObservationConfig ObservationManager::parseObservationConfig(const mc_rtc::Configuration & config,
                                                             const ObservationConfig & defaults) const
{
  ObservationConfig out = defaults;

  out.requestedType = config("type", std::string(""));

  if(out.requestedType.empty())
  {
    mc_rtc::log::error_and_throw("[ObservationManager] Observation entry is missing required field 'type'");
  }

  out.type = out.requestedType;
  out.name = out.requestedType;
  config("name", out.name);

  if(config.has("parameters"))
  {
    out.parameters = config("parameters");
    out.parameters("history", out.history);
    if(out.history <= 0)
      mc_rtc::log::error_and_throw("[ObservationManager] Observation '{}' has invalid history {}", out.name,
                                   out.history);
    if(out.parameters.has("observation_source"))
      out.useRealRobot = parseObservationSource(out.parameters("observation_source").operator std::string(),
                                                "observation '" + out.name + "'");
  }
  else
  {
    out.parameters = emptyConfiguration();
  }

  return out;
}

Eigen::VectorXd ObservationManager::flattenHistory(const Entry & entry) const
{
  int total = 0;

  for(size_t i = 0; i < entry.historyBuffer.size(); ++i)
  {
    total += static_cast<int>(entry.historyBuffer[i].size());
  }

  Eigen::VectorXd out = Eigen::VectorXd::Zero(total);
  if(newest_first_)
  {
    Eigen::Index offset = 0;
    for(const auto & v : entry.historyBuffer)
    {
      out.segment(offset, v.size()) = v;
      offset = offset += v.size();
    }
  }
  else
  {
    Eigen::Index offset = total - entry.historyBuffer[0].size();
    for(const auto & v : entry.historyBuffer)
    {
      out.segment(offset, v.size()) = v;
      offset -= v.size();
    }
  }

  return out;
}

} // namespace rlqp
