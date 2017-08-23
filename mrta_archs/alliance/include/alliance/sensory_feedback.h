#ifndef _ALLIANCE_SENSORY_FEEDBACK_H_
#define _ALLIANCE_SENSORY_FEEDBACK_H_

#include <ros/time.h>

namespace alliance
{
class SensoryFeedback
{
public:
  SensoryFeedback(const SensoryFeedback& sensory_feedback);
  virtual ~SensoryFeedback();
  bool received(ros::Time timestamp = ros::Time::now()) const;

private:
};
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
