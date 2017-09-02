#include "alliance/activity_suppression.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <typeinfo>
#include <utilities/toggle_event.h>

namespace alliance
{
ActivitySuppression::ActivitySuppression(Robot* robot,
                                         BehaviourSet* behaviour_set)
    : Observer::Observer(behaviour_set->getId() + "/activity_suppression"),
      robot_(robot)
{
  suppressed_ = new utilities::functions::UnarySampleHolder(
      behaviour_set->getId() + "/activity_suppression/suppressed",
      ros::Duration(10 * robot_->getTimeoutDuration().toSec()));
}

ActivitySuppression::~ActivitySuppression()
{
  if (suppressed_)
  {
    delete suppressed_;
    suppressed_ = NULL;
  }
  robot_ = NULL;
}

void ActivitySuppression::update(utilities::Event* event)
{
  if (typeid(*event) == typeid(utilities::ToggleEvent))
  {
    ROS_DEBUG_STREAM("Updating " << *suppressed_ << " to "
                                 << ((utilities::ToggleEvent*)event)->getValue()
                                 << ".");
    suppressed_->update((utilities::ToggleEvent*)event);
  }
}

bool ActivitySuppression::isSuppressed(const ros::Time& timestamp) const
{
  return suppressed_->getValue(timestamp);
}
}
