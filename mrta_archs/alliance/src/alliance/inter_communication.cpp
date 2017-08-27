#include "alliance/inter_communication.h"
#include "alliance/robot.h"

namespace alliance
{
InterCommunication::InterCommunication(Robot* robot,
                                       BehaviourSet* behaviour_set)
    : BeaconSignalObserver::BeaconSignalObserver(behaviour_set->getId() +
                                                 "/inter_communication"),
      robot_(robot), task_(behaviour_set->getTask())
{
}

InterCommunication::InterCommunication(
    const InterCommunication& inter_communication)
    : BeaconSignalObserver::BeaconSignalObserver(inter_communication),
      robot_(inter_communication.robot_), task_(inter_communication.task_)
{
}

InterCommunication::~InterCommunication()
{
  robot_ = NULL;
  task_ = NULL;
  std::map<std::string, utilities::functions::UnaryBufferedFunction*>::iterator
      it(robots_.begin());
  while (it != robots_.end())
  {
    if (it->second)
    {
      delete it->second;
      it->second = NULL;
    }
    it++;
  }
  robots_.clear();
}

bool InterCommunication::received(const Robot& robot, const ros::Time& t1,
                                  const ros::Time& t2) const
{
  std::map<std::string,
           utilities::functions::UnaryBufferedFunction*>::const_iterator
      it(robots_.find(robot.getId()));
  if (it == robots_.end())
  {
    return false;
  }
  utilities::functions::UnaryBufferedFunction* function = it->second;
  return function->updated(t1, t2);
}

void InterCommunication::update(utilities::BeaconSignalEvent* event)
{
  if (event->isRelated(*robot_))
  {
    return;
  }
  if (!event->isRelated(*task_))
  {
    ROS_DEBUG_STREAM("Received beacon signal that is not related to "
                     << *task_ << " task.");
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  std::map<std::string, utilities::functions::UnaryBufferedFunction*>::iterator
      it(robots_.find(robot_id));
  utilities::functions::UnaryBufferedFunction* function;
  if (it != robots_.end())
  {
    function = it->second;
  }
  else
  {
    function = new utilities::functions::UnaryBufferedFunction(
        robot_id + "/function", robot_->getMaximumInterruptionDuration(),
        event->getTimestamp());
    robots_[robot_id] = function;
  }
  function->update(event->getTimestamp());
}
}
