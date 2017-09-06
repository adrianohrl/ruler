#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/motivational_behaviour.h"
#include <list>

namespace alliance
{
class BehaviourSet : public BehaviourSetInterface<Robot, BehaviourSet>,
                     public utilities::Subject
{
public:
  BehaviourSet(const RobotPtr& robot, const TaskPtr& task,
               const ros::Duration& buffer_horizon);
  virtual ~BehaviourSet();
  virtual void preProcess();
  MotivationalBehaviourPtr getMotivationalBehaviour() const;
  void setActivationThreshold(double threshold);
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay);
  void setImpatience(double fast_rate);
  void registerActivitySuppression(const BehaviourSetPtr& behaviour_set);
  virtual void setActive(bool active = true,
                         const ros::Time& timestamp = ros::Time::now());

private:
  const MotivationalBehaviourPtr motivational_behaviour_;
  virtual BehaviourSetPtr shared_from_this();
};

typedef boost::shared_ptr<BehaviourSet> BehaviourSetPtr;
typedef boost::shared_ptr<BehaviourSet const> BehaviourSetConstPtr;
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
