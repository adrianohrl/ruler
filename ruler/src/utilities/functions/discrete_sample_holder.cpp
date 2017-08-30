#include "utilities/functions/discrete_sample_holder.h"
#include "utilities/functions/discrete_step_function.h"

namespace utilities
{
namespace functions
{
DiscreteSampleHolder::DiscreteSampleHolder(
    const std::string& id, int value, const ros::Duration& timeout_duration,
    const ros::Duration& buffer_horizon, const ros::Time& start_timestamp)
    : SampleHolder<DiscreteSignalType>::SampleHolder(
          id, new DiscreteStepFunction(value, true), timeout_duration,
          buffer_horizon, start_timestamp)
{
}

DiscreteSampleHolder::DiscreteSampleHolder(const DiscreteSampleHolder& function)
    : SampleHolder<DiscreteSignalType>::SampleHolder(function)
{
}

DiscreteSampleHolder::~DiscreteSampleHolder() {}
}
}
