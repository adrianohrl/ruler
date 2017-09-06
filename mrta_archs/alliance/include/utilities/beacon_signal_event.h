#ifndef _UTILITIES_BEACON_SIGNAL_EVENT_H_
#define _UTILITIES_BEACON_SIGNAL_EVENT_H_

#include "alliance/task.h"
#include <alliance_msgs/BeaconSignal.h>
#include <utilities/event.h>

namespace alliance
{
class Robot;

class BehavedRobot;
}

namespace utilities
{
class BeaconSignalSubject;
typedef boost::shared_ptr<BeaconSignalSubject> BeaconSignalSubjectPtr;
typedef boost::shared_ptr<BeaconSignalSubject const>
    BeaconSignalSubjectConstPtr;

class BeaconSignalEvent : public Event
{
public:
  BeaconSignalEvent(const BeaconSignalSubjectPtr& subject,
                    const alliance_msgs::BeaconSignal& msg);
  BeaconSignalEvent(const BeaconSignalEvent& event);
  virtual ~BeaconSignalEvent();
  alliance_msgs::BeaconSignal getMsg() const;
  bool isRelated(const alliance::Robot& robot) const;
  bool isRelated(const alliance::BehavedRobot& robot) const;
  bool isRelated(const alliance::Task& task) const;

private:
  alliance_msgs::BeaconSignal msg_;
};

typedef boost::shared_ptr<BeaconSignalEvent> BeaconSignalEventPtr;
typedef boost::shared_ptr<BeaconSignalEvent const> BeaconSignalEventConstPtr;
}

#endif // _UTILITIES_BEACON_SIGNAL_EVENT_H_
