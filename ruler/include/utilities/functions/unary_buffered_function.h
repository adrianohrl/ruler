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
                        ros::Time start_timestamp = ros::Time::now());
  UnaryBufferedFunction(std::string id, ros::Duration interruption_delay,
                        ros::Duration buffer_horizon,
                        ros::Time start_timestamp = ros::Time::now());
  UnaryBufferedFunction(const UnaryBufferedFunction& function);
  virtual ~UnaryBufferedFunction();
  bool updated(ros::Time t1, ros::Time t2 = ros::Time::now()) const;
};
}
}

#endif // _UTILITIES_UNARY_BUFFERED_FUNCTION_H_
