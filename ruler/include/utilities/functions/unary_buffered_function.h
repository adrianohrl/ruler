#ifndef _UTILITIES_UNARY_BUFFERED_FUNCTION_H_
#define _UTILITIES_UNARY_BUFFERED_FUNCTION_H_

#include "utilities/unary_signal_type.h"
#include "utilities/functions/buffered_function.h"

namespace utilities
{
namespace functions
{
class UnaryBufferedFunction : public BufferedFunction<UnarySignalType>
{
public:
  UnaryBufferedFunction(std::string id, ros::Duration interruption_delay,
                        ros::Duration buffer_horizon);
  UnaryBufferedFunction(const UnaryBufferedFunction& function);
  virtual ~UnaryBufferedFunction();
};
}
}

#endif // _UTILITIES_UNARY_BUFFERED_FUNCTION_H_
