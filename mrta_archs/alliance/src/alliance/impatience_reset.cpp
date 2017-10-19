#include "alliance/impatience_reset.h"
#include "alliance/robot.h"

namespace alliance
{
ImpatienceReset::ImpatienceReset() : resetted_(false) {}

ImpatienceReset::~ImpatienceReset() {}

void ImpatienceReset::init(const InterRobotCommunicationPtr& monitor)
{
  if (!monitor_)
  {
    monitor_ = monitor;
  }
}

bool ImpatienceReset::isResetted(const ros::Time& timestamp)
{
  if (resetted_)
  {
    return false;
  }
  resetted_ =
      monitor_->received(monitor_->getLastUpdateTimestamp(), timestamp) &&
      !monitor_->received(ros::Time(), monitor_->getLastUpdateTimestamp());
  return resetted_;
}
}
