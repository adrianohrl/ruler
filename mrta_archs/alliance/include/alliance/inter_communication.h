#ifndef _ALLIANCE_INTER_COMMUNICATION_H_
#define _ALLIANCE_INTER_COMMUNICATION_H_

//#include <list>
#include "utilities/beacon_signal_observer.h"

namespace alliance
{
class Robot;

class BehaviourSet;

class InterCommunication : public utilities::BeaconSignalObserver
{
public:
  InterCommunication(Robot *robot, BehaviourSet* behaviour_set);
  InterCommunication(const InterCommunication& inter_communication);
  virtual ~InterCommunication();
  bool received(ros::Time timestamp = ros::Time::now()) const;
  virtual void update(utilities::BeaconSignalEvent *event);

private:
  Robot* robot_;
  std::list<Robot*> robots_;
  Robot* get(std::string robot_id) const;
};
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
