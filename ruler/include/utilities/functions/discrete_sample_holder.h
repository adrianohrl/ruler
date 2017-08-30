#ifndef _UTILITIES_DISCRETE_SAMPLE_HOLDER_H_
#define _UTILITIES_DISCRETE_SAMPLE_HOLDER_H_

#include "utilities/discrete_signal_type.h"
#include "utilities/functions/sample_holder.h"

namespace utilities
{
namespace functions
{
class DiscreteSampleHolder : public SampleHolder<DiscreteSignalType>
{
public:
  DiscreteSampleHolder(const std::string& id, int value,
                       const ros::Duration& timeout_duration,
                       const ros::Duration& buffer_horizon,
                       const ros::Time& start_timestamp = ros::Time::now());
  DiscreteSampleHolder(const DiscreteSampleHolder& function);
  virtual ~DiscreteSampleHolder();
};
}
}

#endif // _UTILITIES_DISCRETE_SAMPLE_HOLDER_H_
