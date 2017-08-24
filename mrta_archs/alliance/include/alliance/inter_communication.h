#ifndef _ALLIANCE_INTER_COMMUNICATION_H_
#define _ALLIANCE_INTER_COMMUNICATION_H_

#include "utilities/beacon_signal_observer.h"

namespace alliance
{
class Robot;

class InterCommunication : public utilities::BeaconSignalObserver
{
public:
  InterCommunication(Robot *robot);
  InterCommunication(const InterCommunication& inter_communication);
  virtual ~InterCommunication();
  bool received(ros::Time timestamp = ros::Time::now()) const;
  virtual void update(utilities::BeaconSignalEvent *event);

private:
  Robot* robot_;
};
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
