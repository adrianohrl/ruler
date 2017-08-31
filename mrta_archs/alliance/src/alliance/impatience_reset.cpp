#include "alliance/impatience_reset.h"
#include "alliance/robot.h"

namespace alliance
{
ImpatienceReset::ImpatienceReset(InterCommunication* monitor)
    : monitor_(monitor)
{
}

ImpatienceReset::ImpatienceReset(const ImpatienceReset& impatience_reset)
    : monitor_(impatience_reset.monitor_)
{
}

ImpatienceReset::~ImpatienceReset() { monitor_ = NULL; }

bool ImpatienceReset::isResetted(const ros::Time& timestamp) const
{
  return !(monitor_->received(monitor_->getLastUpdateTimestamp(), timestamp) &&
           !monitor_->received(ros::Time(),
                               monitor_->getLastUpdateTimestamp()));
}
}
