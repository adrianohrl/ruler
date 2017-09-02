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
  virtual ~InterCommunication();
  bool received(const ros::Time& t1,
                const ros::Time& t2) const;
  bool received(const std::string& robot_id, const ros::Time& t1,
                const ros::Time& t2) const;
  virtual void update(utilities::BeaconSignalEvent* event);
  ros::Time getLastUpdateTimestamp() const;

private:
  Robot* robot_;
  Task* task_;
  ros::Time last_update_timestamp_;
  std::map<std::string, utilities::functions::UnarySampleHolder*> robots_;
};
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
