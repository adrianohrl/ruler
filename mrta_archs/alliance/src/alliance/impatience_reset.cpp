#include "alliance/impatience_reset.h"
#include "alliance/robot.h"

namespace alliance
{
ImpatienceReset::ImpatienceReset(const InterCommunicationPtr& monitor)
    : monitor_(monitor)
{
}

ImpatienceReset::~ImpatienceReset() {}

bool ImpatienceReset::isResetted(const ros::Time& timestamp) const
{
  return !(monitor_->received(monitor_->getLastUpdateTimestamp(), timestamp) &&
           !monitor_->received(ros::Time(),
                               monitor_->getLastUpdateTimestamp()));
}
}
