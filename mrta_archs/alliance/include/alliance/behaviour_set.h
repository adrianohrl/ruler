#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/motivational_behaviour.h"
#include <list>

namespace alliance
{
class BehaviourSet : public BehaviourSetInterface<Robot>, public utilities::Subject
{
public:
  BehaviourSet(Robot* robot, Task* task);
  virtual ~BehaviourSet();
  void process();
  bool isActive(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Time getActivationTimestamp() const;
  MotivationalBehaviour* getMotivationalBehaviour() const;
  void setActive(bool active = true,
                 const ros::Time& timestamp = ros::Time::now());
  void setActivationThreshold(double threshold);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void registerActivitySuppression(BehaviourSet* behaviour_set);

private:
  ros::Time activation_timestamp_;
  utilities::functions::UnarySampleHolder* active_;
  MotivationalBehaviour* motivational_behaviour_;
};
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
