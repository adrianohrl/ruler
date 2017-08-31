#include "alliance/inter_communication.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"

namespace alliance
{
InterCommunication::InterCommunication(Robot* robot,
                                       BehaviourSet* behaviour_set)
    : BeaconSignalObserver::BeaconSignalObserver(behaviour_set->getId() +
                                                 "/inter_communication"),
      robot_(robot), task_(behaviour_set->getTask()),
      last_update_timestamp_(ros::Time::now())
{
}

InterCommunication::InterCommunication(const InterCommunication& monitor)
    : BeaconSignalObserver::BeaconSignalObserver(monitor),
      robot_(monitor.robot_), task_(monitor.task_),
      last_update_timestamp_(monitor.last_update_timestamp_)
{
}

InterCommunication::~InterCommunication()
{
  robot_ = NULL;
  task_ = NULL;
  std::map<std::string, utilities::functions::UnarySampleHolder*>::iterator it(
      robots_.begin());
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

bool InterCommunication::received(const ros::Time& t1,
                                  const ros::Time& t2) const
{
  std::map<std::string,
           utilities::functions::UnarySampleHolder*>::const_iterator
      it(robots_.begin());
  while (it != robots_.end())
  {
    utilities::functions::UnarySampleHolder* sample_holder = it->second;
    if (sample_holder->updated(t1, t2))
    {
      return true;
    }
    it++;
  }
  return false;
}

bool InterCommunication::received(const std::string& robot_id,
                                  const ros::Time& t1,
                                  const ros::Time& t2) const
{
  std::map<std::string,
           utilities::functions::UnarySampleHolder*>::const_iterator
      it(robots_.find(robot_id));
  if (it == robots_.end())
  {
    return false;
  }
  utilities::functions::UnarySampleHolder* sample_holder = it->second;
  return sample_holder->updated(t1, t2);
}

void InterCommunication::update(utilities::BeaconSignalEvent* event)
{
  if (event->isRelated(*robot_) || !event->isRelated(*task_))
  {
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  std::map<std::string, utilities::functions::UnarySampleHolder*>::iterator it(
      robots_.find(robot_id));
  utilities::functions::UnarySampleHolder* sample_holder = it->second;
  if (it == robots_.end())
  {
    sample_holder = new utilities::functions::UnarySampleHolder(
        robot_id + "/sample_holder", robot_->getTimeoutDuration(),
        ros::Duration(10 * robot_->getTimeoutDuration().toSec()),
        event->getTimestamp());
    robots_[robot_id] = sample_holder;
  }
  ROS_DEBUG_STREAM("Updating " << *sample_holder << ".");
  sample_holder->update(event->getTimestamp());
  last_update_timestamp_ = event->getTimestamp();
}

ros::Time InterCommunication::getLastUpdateTimestamp() const
{
  return last_update_timestamp_;
}
}
