#ifndef _ALLIANCE_SENSORY_FEEDBACK_H_
#define _ALLIANCE_SENSORY_FEEDBACK_H_

#include <ros/time.h>

namespace alliance
{
class SensoryFeedback
{
public:
  SensoryFeedback();
  SensoryFeedback(const SensoryFeedback& sensory_feedback);
  virtual ~SensoryFeedback();
  bool received(const ros::Time& timestamp = ros::Time::now()) const;
};
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
