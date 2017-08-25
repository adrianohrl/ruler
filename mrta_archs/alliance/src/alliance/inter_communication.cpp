#include "alliance/inter_communication.h"
#include "alliance/robot.h"

namespace alliance
{
InterCommunication::InterCommunication(Robot* robot, BehaviourSet *behaviour_set)
    : BeaconSignalObserver::BeaconSignalObserver(behaviour_set->getId() +
                                                 "/inter_communication"),
      robot_(robot)
{
}

InterCommunication::InterCommunication(
    const InterCommunication& inter_communication)
    : BeaconSignalObserver::BeaconSignalObserver(inter_communication),
      robot_(inter_communication.robot_)
{
}

InterCommunication::~InterCommunication()
{
  robot_ = NULL;
  std::list<Robot*>::iterator it(robots_.begin());
  while (it != robots_.end())
  {
    if (*it)
    {
      *it = NULL;
    }
    it++;
  }
  robots_.clear();
}

bool InterCommunication::received(ros::Time timestamp) const { return false; }

void InterCommunication::update(utilities::BeaconSignalEvent* event)
{
  if (event->isRelated(*robot_))
  {
    return;
  }
  std::string robot_id(event->getMsg().header.frame_id);
  Robot* robot = get(robot_id);
  if (!robot);
  {
    robot = new Robot(robot_id, robot_id);
  }
  // update robot
}

Robot *InterCommunication::get(std::string robot_id) const
{
  std::list<Robot*>::const_iterator it(robots_.begin());
  while (it != robots_.end())
  {
    Robot* robot = *it;
    if (robot->getId() == robot_id)
    {
      return robot;
    }
    it++;
  }
  return NULL;
}
}
