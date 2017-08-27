#include "utilities/functions/unary_buffered_function.h"
#include "utilities/functions/unary_step_function.h"

namespace utilities
{
namespace functions
{
UnaryBufferedFunction::UnaryBufferedFunction(std::string id,
                                             ros::Duration interruption_delay,
                                             ros::Time start_timestamp)
    : BufferedFunction<UnarySignalType>::BufferedFunction(
          id, new UnaryStepFunction(0.0, true), interruption_delay,
          ros::Duration(5 * interruption_delay.toSec()), start_timestamp)
{
}

UnaryBufferedFunction::UnaryBufferedFunction(std::string id,
                                             ros::Duration interruption_delay,
                                             ros::Duration buffer_horizon,
                                             ros::Time start_timestamp)
    : BufferedFunction<UnarySignalType>::BufferedFunction(
          id, new UnaryStepFunction(0.0, true), interruption_delay,
          buffer_horizon, start_timestamp)
{
}

UnaryBufferedFunction::UnaryBufferedFunction(
    const UnaryBufferedFunction& function)
    : BufferedFunction<UnarySignalType>::BufferedFunction(function)
{
}

UnaryBufferedFunction::~UnaryBufferedFunction() {}

bool UnaryBufferedFunction::updated(ros::Time t1, ros::Time t2) const
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
  ros::Duration delta_d(BufferedFunction<UnarySignalType>::getInterruptionDelay().toSec() / 2.5);
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
