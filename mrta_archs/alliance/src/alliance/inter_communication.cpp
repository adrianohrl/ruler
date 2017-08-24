#include "alliance/inter_communication.h"
#include "alliance/robot.h"

namespace alliance
{
InterCommunication::InterCommunication(Robot* robot)
    : BeaconSignalObserver::BeaconSignalObserver(robot->getId() +
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

InterCommunication::~InterCommunication() { robot_ = NULL; }

bool InterCommunication::received(ros::Time timestamp) const { return false; }

void InterCommunication::update(utilities::BeaconSignalEvent* event)
{
  if (event->isRelated(*robot_))
  {
    return;
  }
  ROS_INFO_STREAM("Updating the " << *this << "observer.");
}
}
