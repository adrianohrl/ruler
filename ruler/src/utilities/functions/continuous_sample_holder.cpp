#include "utilities/functions/continuous_sample_holder.h"
#include "utilities/functions/continuous_step_function.h"

namespace utilities
{
namespace functions
{
ContinuousSampleHolder::ContinuousSampleHolder(
    const std::string& id, double value, const ros::Duration& buffer_horizon,
    const ros::Time& start_timestamp)
    : SampleHolder<ContinuousSignalType>::SampleHolder(
          id, new ContinuousStepFunction(value, true), buffer_horizon,
          start_timestamp)
{
  ROS_WARN("[CSH] constructed");
}

ContinuousSampleHolder::ContinuousSampleHolder(
    const std::string& id, double value, const ros::Duration& timeout_duration,
    const ros::Duration& buffer_horizon, const ros::Time& start_timestamp)
    : SampleHolder<ContinuousSignalType>::SampleHolder(
          id, new ContinuousStepFunction(value, true), timeout_duration,
          buffer_horizon, start_timestamp)
{
}

ContinuousSampleHolder::ContinuousSampleHolder(
    const ContinuousSampleHolder& function)
    : SampleHolder<ContinuousSignalType>::SampleHolder(function)
{
}

ContinuousSampleHolder::~ContinuousSampleHolder() {}
}
}
