#include "alliance/inter_communication.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"

namespace alliance
{
InterCommunication::InterCommunication(const RobotPtr &robot,
                                       const BehaviourSetPtr& behaviour_set)
    : BeaconSignalObserver::BeaconSignalObserver(behaviour_set->getId() +
                                                 "/inter_communication"),
      robot_(robot), task_(behaviour_set->getTask()),
      last_update_timestamp_(ros::Time::now())
{
}

InterCommunication::~InterCommunication()
{
}

bool InterCommunication::received(const ros::Time& t1,
                                  const ros::Time& t2) const
{
  std::map<std::string,
           utilities::functions::UnarySampleHolderPtr>::const_iterator
      it(robots_.begin());
  utilities::functions::UnarySampleHolderPtr sample_holder;
  while (it != robots_.end())
  {
    sample_holder = it->second;
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
           utilities::functions::UnarySampleHolderPtr>::const_iterator
      it(robots_.find(robot_id));
  if (it == robots_.end())
  {
    return false;
  }
  utilities::functions::UnarySampleHolderPtr sample_holder(it->second);
  return sample_holder->updated(t1, t2);
}

void InterCommunication::update(const utilities::BeaconSignalEventConstPtr& event)
{
  if (event->isRelated(*robot_) || !event->isRelated(*task_))
  {
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  std::map<std::string, utilities::functions::UnarySampleHolderPtr>::iterator it(
      robots_.find(robot_id));
  utilities::functions::UnarySampleHolderPtr sample_holder(it->second);
  if (it == robots_.end())
  {
    sample_holder.reset(new utilities::functions::UnarySampleHolder(
        robot_id + "/sample_holder", robot_->getTimeoutDuration(),
        ros::Duration(10 * robot_->getTimeoutDuration().toSec()),
        event->getTimestamp()));
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
