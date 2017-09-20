#ifndef _ALLIANCE_INTER_COMMUNICATION_H_
#define _ALLIANCE_INTER_COMMUNICATION_H_

#include <map>
#include "alliance/task.h"
#include <utilities/functions/unary_sample_holder.h>
#include <utilities/beacon_signal_observer.h>

namespace alliance
{
class Robot;
typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<Robot const> RobotConstPtr;

class BehaviourSet;
typedef boost::shared_ptr<BehaviourSet> BehaviourSetPtr;
typedef boost::shared_ptr<BehaviourSet const> BehaviourSetConstPtr;

class InterCommunication : public utilities::BeaconSignalObserver
{
public:
  InterCommunication(const RobotPtr& robot,
                     const BehaviourSetPtr& behaviour_set);
  virtual ~InterCommunication();
  bool received(const ros::Time& t1, const ros::Time& t2) const;
  bool received(const std::string& robot_id, const ros::Time& t1,
                const ros::Time& t2) const;
  virtual void update(const utilities::BeaconSignalEventConstPtr& event);
  ros::Time getLastUpdateTimestamp() const;

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  typedef std::map<std::string, SampleHolderPtr>::iterator iterator;
  typedef std::map<std::string, SampleHolderPtr>::const_iterator const_iterator;
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  ros::Time last_update_timestamp_;
  std::map<std::string, SampleHolderPtr> robots_;
};

typedef boost::shared_ptr<InterCommunication> InterCommunicationPtr;
typedef boost::shared_ptr<InterCommunication const> InterCommunicationConstPtr;
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
