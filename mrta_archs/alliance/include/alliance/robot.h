#ifndef _ALLIANCE_ROBOT_H_
#define _ALLIANCE_ROBOT_H_

#include <list>
#include <ros/time.h>
#include <ros/rate.h>
#include "alliance/acquiescence.h"
#include "alliance/behaviour_set.h"
#include <utilities/has_name.h>

namespace alliance
{
class Robot : public utilities::HasName
{
public:
  Robot(const std::string& id, const std::string& name);
  Robot(const Robot& robot);
  virtual ~Robot();
  void process();
  bool isActive() const;
  ros::Rate getBroadcastRate() const;
  ros::Duration getTimeoutDuration() const;
  std::list<BehaviourSet*> getBehaviourSets() const;
  Task* getExecutingTask() const;
  void setBroadcastRate(const ros::Rate& broadcast_rate);
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  void addBehaviourSet(BehaviourSet* behaviour_set);

private:
  ros::Rate broadcast_rate_;
  ros::Duration timeout_duration_;
  Acquiescence* acquiescence_;
  BehaviourSet* active_behaviour_set_;
  std::list<BehaviourSet*> behaviour_sets_;
  bool contains(const BehaviourSet& behaviour_set) const;
};
}

#endif // _ALLIANCE_ROBOT_H_
