#ifndef _ALLIANCE_ROBOT_H_
#define _ALLIANCE_ROBOT_H_

#include <list>
#include <ros/time.h>
#include <ros/rate.h>
#include "alliance/acquiescence.h"
#include "alliance/behaviour_set.h"
#include "alliance/impatience.h"
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
  bool received(const Robot& robot, const Task& task, const ros::Time& t1,
                const ros::Time& t2);
  bool isActive() const;
  ros::Rate getBroadcastRate() const;
  ros::Duration getMaximumInterruptionDuration() const;
  double getImpatience(const ros::Time& timestamp = ros::Time::now()) const;
  bool isAcquiescent(const ros::Time& timestamp = ros::Time::now()) const;
  std::list<BehaviourSet*> getBehaviourSets() const;
  Task* getExecutingTask() const;
  void setBroadcastRate(const ros::Rate& broadcast_rate);
  void setMaximumInterruptionDuration(
      const ros::Duration& max_interruption_duration);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void setImpatience(const Robot& robot, double slow_rate,
                     const ros::Duration& reliability_duration);
  void addBehaviourSet(BehaviourSet* behaviour_set);

private:
  ros::Rate broadcast_rate_;
  ros::Duration max_interruption_duration_;
  Acquiescence* acquiescence_;
  BehaviourSet* active_behaviour_set_;
  std::list<BehaviourSet*> behaviour_sets_;
  Impatience* impatience_;
  bool contains(const BehaviourSet& behaviour_set) const;
};
}

#endif // _ALLIANCE_ROBOT_H_
