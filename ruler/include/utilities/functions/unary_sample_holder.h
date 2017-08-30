#ifndef _UTILITIES_UNARY_SAMPLE_HOLDER_H_
#define _UTILITIES_UNARY_SAMPLE_HOLDER_H_

#include "utilities/unary_signal_type.h"
#include "utilities/functions/sample_holder.h"

namespace utilities
{
namespace functions
{
class UnarySampleHolder : public SampleHolder<UnarySignalType>
{
public:
  UnarySampleHolder(std::string id, ros::Duration timeout_duration,
                    ros::Time start_timestamp = ros::Time::now());
  UnarySampleHolder(std::string id, ros::Duration timeout_duration,
                    ros::Duration buffer_horizon,
                    ros::Time start_timestamp = ros::Time::now());
  UnarySampleHolder(const UnarySampleHolder& function);
  virtual ~UnarySampleHolder();
  using BufferedFunction<UnarySignalType>::update;
  using SampleHolder<UnarySignalType>::update;
  bool updated(ros::Time t1, ros::Time t2 = ros::Time::now()) const;
};
}
}

#endif // _UTILITIES_UNARY_SAMPLE_HOLDER_H_
