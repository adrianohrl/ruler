#include "utilities/functions/unary_sample_holder.h"
#include "utilities/functions/unary_step_function.h"

namespace utilities
{
namespace functions
{
UnarySampleHolder::UnarySampleHolder(std::string id,
                                     ros::Duration timeout_duration,
                                     ros::Time start_timestamp)
    : SampleHolder<UnarySignalType>::SampleHolder(
          id, new UnaryStepFunction(0.0, true), timeout_duration,
          ros::Duration(5 * timeout_duration.toSec()), start_timestamp)
{
}

UnarySampleHolder::UnarySampleHolder(std::string id,
                                     ros::Duration timeout_duration,
                                     ros::Duration buffer_horizon,
                                     ros::Time start_timestamp)
    : SampleHolder<UnarySignalType>::SampleHolder(
          id, new UnaryStepFunction(0.0, true), timeout_duration,
          buffer_horizon, start_timestamp)
{
}

UnarySampleHolder::UnarySampleHolder(const UnarySampleHolder& function)
    : SampleHolder<UnarySignalType>::SampleHolder(function)
{
}

UnarySampleHolder::~UnarySampleHolder() {}

bool UnarySampleHolder::updated(ros::Time t1, ros::Time t2) const
{
  ros::Time t0(BufferedFunction<UnarySignalType>::getStartTimestamp());
  if (t0 > t1)
  {
    throw utilities::Exception("t1 must be greater than t0.");
  }
  if (t1 > t2)
  {
    throw utilities::Exception("t2 must be greater than t1.");
  }
  ros::Duration d1(t1 - t0);
  ros::Duration d2(t2 - t0);
  ros::Duration delta_d(
      BufferedFunction<UnarySignalType>::getTimeoutDuration().toSec() / 2.5);
  for (ros::Duration d(d1); d <= d2; d += delta_d)
  {
    if (BufferedFunction<UnarySignalType>::getValue(d.toSec()))
    {
      return true;
    }
  }
  return false;
}
}
}
