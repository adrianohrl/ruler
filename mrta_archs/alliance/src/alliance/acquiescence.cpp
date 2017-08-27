#include "alliance/acquiescence.h"

namespace alliance
{
Acquiescence::Acquiescence(const ros::Duration& yielding_delay,
                           const ros::Duration& giving_up_delay)
    : yielding_delay_(yielding_delay), giving_up_delay_(giving_up_delay)
{
}

Acquiescence::Acquiescence(const Acquiescence& acquiescence)
    : yielding_delay_(acquiescence.yielding_delay_),
      giving_up_delay_(acquiescence.giving_up_delay_)
{
}

Acquiescence::~Acquiescence() {}

ros::Duration Acquiescence::getYieldingDelay() const { return yielding_delay_; }

ros::Duration Acquiescence::getGivingUpDelay() const
{
  return giving_up_delay_;
}

bool Acquiescence::isAcquiescent(const ros::Time& timestamp) { return false; }

void Acquiescence::setYieldingDelay(const ros::Duration& yielding_delay)
{
  yielding_delay_ = yielding_delay;
}

void Acquiescence::setGivingUpDelay(const ros::Duration& giving_up_delay)
{
  giving_up_delay_ = giving_up_delay;
}
}
