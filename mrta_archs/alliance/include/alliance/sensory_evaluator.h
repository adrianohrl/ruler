#ifndef _ALLIANCE_SENSORY_EVALUATOR_H_
#define _ALLIANCE_SENSORY_EVALUATOR_H_

#include "alliance/behaved_robot.h"
#include "alliance/sensor.h"
#include <list>
#include <ros/publisher.h>

namespace alliance
{
class SensoryEvaluator
{
public:
  SensoryEvaluator();
  virtual ~SensoryEvaluator();
  virtual void initialize(const ros::NodeHandlePtr& nh,
                          const BehavedRobot& robot, const Task& task,
                          const std::list<SensorPtr>& sensors);
  void process();
  virtual bool isApplicable() = 0;

protected:
  typedef std::list<SensorPtr>::iterator iterator;
  typedef std::list<SensorPtr>::const_iterator const_iterator;
  std::list<SensorPtr> sensors_;

private:
  ros::NodeHandlePtr nh_;
  ros::Publisher sensory_feedback_pub_;
  alliance_msgs::SensoryFeedback sensory_feedback_msg_;
};
}

#endif // _ALLIANCE_SENSORY_EVALUATOR_H_
