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
  SensoryFeedback(Task* task);
  SensoryFeedback(const SensoryFeedback& sensory_feedback);
  virtual ~SensoryFeedback();
  bool isApplicable(const ros::Time& timestamp = ros::Time::now()) const;

private:
  Task* task_;
  std::list<Sensor*> sensors_;
};
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
