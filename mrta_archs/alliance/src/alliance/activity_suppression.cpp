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
      robot_(robot), last_accession_timestamp_(ros::Time::now()),
      last_suppression_timestamp_(ros::Time())
{
}

ActivitySuppression::ActivitySuppression(
    const ActivitySuppression& activity_suppression)
    : Observer::Observer(activity_suppression),
      robot_(activity_suppression.robot_),
      last_accession_timestamp_(activity_suppression.last_accession_timestamp_),
      last_suppression_timestamp_(
          activity_suppression.last_suppression_timestamp_)
{
}

ActivitySuppression::~ActivitySuppression() { robot_ = NULL; }

void ActivitySuppression::update(utilities::Event* event)
{
  // estou pensando em tirar esse observer pattern daki
  // pq praticamente o robo eh quem faz esse controle (totalmente)
  // dai a unica coisa que esta classe tem que fazer eh o controle temporal
  // desta variavel

  if (typeid(*event) == typeid(utilities::ToggleEvent))
  {
    utilities::ToggleEvent* toggle_event = (utilities::ToggleEvent*)event;
    ros::Time timestamp(toggle_event->getTimestamp());
    if (toggle_event->getValue() == isSuppressed(timestamp))
    {
      return;
    }
    if (toggle_event->getValue())
    {
      if (last_accession_timestamp_ < timestamp)
      {
        last_accession_timestamp_ = timestamp;
      }
    }
    else if (last_suppression_timestamp_ < timestamp)
    {
      last_suppression_timestamp_ = timestamp;
    }
  }
}

bool ActivitySuppression::isSuppressed(const ros::Time& timestamp) const
{
  if (timestamp < last_suppression_timestamp_ &&
      timestamp < last_accession_timestamp_)
  {
    throw utilities::Exception("Too late.");
  }
  return timestamp < last_suppression_timestamp_ &&
             timestamp >= last_accession_timestamp_ ||
         timestamp >= last_suppression_timestamp_ &&
             timestamp < last_accession_timestamp_;
}
}
