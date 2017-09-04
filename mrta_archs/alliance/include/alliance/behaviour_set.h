#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/motivational_behaviour.h"
#include <list>

namespace alliance
{
class BehaviourSet : public BehaviourSetInterface<Robot>,
                     public utilities::Subject
{
public:
  BehaviourSet(Robot* robot, Task* task, ros::Duration buffer_horizon);
  virtual ~BehaviourSet();
  virtual void preProcess();
  MotivationalBehaviour* getMotivationalBehaviour() const;
  void setActivationThreshold(double threshold);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void registerActivitySuppression(BehaviourSet* behaviour_set);
  virtual void setActive(bool active = true,
                         const ros::Time& timestamp = ros::Time::now());

private:
  MotivationalBehaviour* motivational_behaviour_;
};
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
