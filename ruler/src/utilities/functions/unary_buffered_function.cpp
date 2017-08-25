#include "utilities/functions/unary_buffered_function.h"
#include "utilities/functions/unary_step_function.h"

namespace utilities
{
namespace functions
{
UnaryBufferedFunction::UnaryBufferedFunction(std::string id,
                                             ros::Duration interruption_delay,
                                             ros::Duration buffer_horizon)
    : BufferedFunction<UnarySignalType>::BufferedFunction(
          id, new UnaryStepFunction(0.0, true), interruption_delay, buffer_horizon)
{
}

UnaryBufferedFunction::UnaryBufferedFunction(
    const UnaryBufferedFunction& function)
    : BufferedFunction<UnarySignalType>::BufferedFunction(function)
{
}

UnaryBufferedFunction::~UnaryBufferedFunction() {}
}
}
