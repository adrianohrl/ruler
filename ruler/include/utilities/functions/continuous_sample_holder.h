#ifndef _UTILITIES_CONTINUOUS_SAMPLE_HOLDER_H_
#define _UTILITIES_CONTINUOUS_SAMPLE_HOLDER_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/functions/sample_holder.h"

namespace utilities
{
namespace functions
{
class ContinuousSampleHolder : public SampleHolder<ContinuousSignalType>
{
public:
  ContinuousSampleHolder(const std::string& id, double value,
                       const ros::Duration& buffer_horizon,
                       const ros::Time& start_timestamp = ros::Time::now());
  ContinuousSampleHolder(const std::string& id, double value,
                       const ros::Duration& timeout_duration,
                       const ros::Duration& buffer_horizon,
                       const ros::Time& start_timestamp = ros::Time::now());
  ContinuousSampleHolder(const ContinuousSampleHolder& function);
  virtual ~ContinuousSampleHolder();
};
}
}

#endif // _UTILITIES_CONTINUOUS_SAMPLE_HOLDER_H_
