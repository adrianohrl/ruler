#include "alliance/behaved_robot.h"
#include "alliance/robot.h"
#include "utilities/beacon_signal_event.h"
#include "utilities/beacon_signal_subject.h"

namespace utilities
{
BeaconSignalEvent::BeaconSignalEvent(const BeaconSignalSubjectPtr& subject,
                                     const alliance_msgs::BeaconSignal& msg)
    : Event::Event(subject, msg.header.stamp),
      msg_(msg)
{
}

BeaconSignalEvent::BeaconSignalEvent(const BeaconSignalEvent& event)
    : Event::Event(event)
{
}

BeaconSignalEvent::~BeaconSignalEvent() {}

alliance_msgs::BeaconSignal BeaconSignalEvent::getMsg() const { return msg_; }

bool BeaconSignalEvent::isRelated(const alliance::Robot& robot) const
{
  return msg_.header.frame_id == robot.getId();
}

bool BeaconSignalEvent::isRelated(const alliance::BehavedRobot& robot) const
{
  return msg_.header.frame_id == robot.getId();
}

bool BeaconSignalEvent::isRelated(const alliance::Task& task) const
{
  return msg_.task_id == task.getId();
}
}
