#ifndef _ALLIANCE_SENSORY_FEEDBACK_H_
#define _ALLIANCE_SENSORY_FEEDBACK_H_

#include "alliance/sensor.h"
#include "alliance/task.h"
#include <list>
#include <ros/time.h>

namespace alliance
{
class SensoryFeedback
{
public:
  SensoryFeedback(const TaskPtr& task);
  virtual ~SensoryFeedback();
  bool isApplicable(const ros::Time& timestamp = ros::Time::now()) const;

private:
  const TaskPtr task_;
  const std::list<SensorPtr> sensors_;
};

typedef boost::shared_ptr<SensoryFeedback> SensoryFeedbackPtr;
typedef boost::shared_ptr<SensoryFeedback const> SensoryFeedbackConstPtr;
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
