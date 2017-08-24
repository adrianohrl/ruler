#include "alliance/activity_suppression.h"
#include "alliance/robot.h"
#include <typeinfo>
#include <utilities/toggle_event.h>

namespace alliance
{
ActivitySuppression::ActivitySuppression(Robot* robot, BehaviourSet *behaviour_set)
    : Observer::Observer(behaviour_set->getId() + "/activity_suppression"),
      robot_(robot)
{
}

ActivitySuppression::ActivitySuppression(
    const ActivitySuppression& activity_suppression)
    : Observer::Observer(activity_suppression),
      robot_(activity_suppression.robot_)
{
}

ActivitySuppression::~ActivitySuppression() { robot_ = NULL; }

void ActivitySuppression::update(utilities::Event* event)
{
  if (typeid(*event) == typeid(utilities::ToggleEvent))
  {
    utilities::ToggleEvent* toggle_event = (utilities::ToggleEvent*) event;
    ROS_INFO_STREAM(
        "[Updating activity_suppresion] active: " << toggle_event->getValue());
  }
}

bool ActivitySuppression::suppress(ros::Time timestamp) const { return false; }
}
