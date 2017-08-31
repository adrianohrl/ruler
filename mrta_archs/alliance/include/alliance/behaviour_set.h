#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include "alliance/layer.h"
#include "alliance/motivational_behaviour.h"
#include "alliance/task.h"
#include <list>
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
class BehaviourSet : public utilities::Subject
{
public:
  BehaviourSet(Robot* robot, Task* task);
  BehaviourSet(const BehaviourSet& behaviour_set);
  virtual ~BehaviourSet();
  void process();
  MotivationalBehaviour* getMotivationalBehaviour() const;
  bool isActive(const ros::Time &timestamp = ros::Time::now()) const;
  Task* getTask() const;
  ros::Time getActivationTimestamp() const;
  void setActive(bool active = true, const ros::Time &timestamp = ros::Time::now());
  void setActivationThreshold(double threshold);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void registerActivitySuppression(BehaviourSet* behaviour_set);
  bool operator==(const BehaviourSet& behaviour_set) const;
  bool operator!=(const BehaviourSet& behaviour_set) const;

private:
  ros::Time activation_timestamp_;
  utilities::functions::UnarySampleHolder* active_;
  MotivationalBehaviour* motivational_behaviour_;
  Task* task_;
};
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
