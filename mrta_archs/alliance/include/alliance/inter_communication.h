#ifndef _ALLIANCE_INTER_COMMUNICATION_H_
#define _ALLIANCE_INTER_COMMUNICATION_H_

#include <map>
#include "alliance/task.h"
#include <utilities/functions/unary_sample_holder.h>
#include <utilities/beacon_signal_observer.h>

namespace alliance
{
class Robot;

class BehaviourSet;

class InterCommunication : public utilities::BeaconSignalObserver
{
public:
  InterCommunication(Robot* robot, BehaviourSet* behaviour_set);
  InterCommunication(const InterCommunication& inter_communication);
  virtual ~InterCommunication();
  bool received(const std::string& robot_id, const ros::Time& t1,
                const ros::Time& t2) const;
  virtual void update(utilities::BeaconSignalEvent* event);

private:
  Robot* robot_;
  Task* task_;
  std::map<std::string, utilities::functions::UnarySampleHolder*> robots_;
};
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
