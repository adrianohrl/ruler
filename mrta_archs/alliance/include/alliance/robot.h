#ifndef _ALLIANCE_ROBOT_H_
#define _ALLIANCE_ROBOT_H_

#include <list>
#include <ros/time.h>
#include "alliance/acquiescence.h"
#include "alliance/behaviour_set.h"
#include "alliance/impatience.h"
#include <utilities/has_name.h>

namespace alliance
{
class Robot : public utilities::HasName
{
public: 
  Robot(std::string id, std::string name);
  Robot(const Robot& robot);
  virtual ~Robot();
  bool isActive() const;
  double getBroadcastRate() const;
  ros::Duration getQuietDuration() const;
  double getImpatience(ros::Time timestamp = ros::Time::now()) const;
  bool isAcquiescent(ros::Time timestamp = ros::Time::now()) const;
  std::list<BehaviourSet*> getBehaviourSets() const;
  Task* getExecutingTask() const;
  void setBroadcastRate(double broadcast_rate);
  void setQuietDuration(ros::Duration quiet_duration);
  void setAcquiescence(ros::Duration yielding_delay,
                       ros::Duration giving_up_delay);
  void setImpatience(double fast_rate);
  void setImpatience(const Robot& robot, double slow_rate, ros::Duration reliability_duration);
  void addBehaviourSet(BehaviourSet* behaviour_set);

private:
  double broadcast_rate_;
  ros::Duration quiet_duration_;
  Acquiescence* acquiescence_;
  BehaviourSet* active_behaviour_set_;
  std::list<BehaviourSet*> behaviour_sets_;
  Impatience* impatience_;
  bool contains(const BehaviourSet& behaviour_set) const;
};
}

#endif // _ALLIANCE_ROBOT_H_
