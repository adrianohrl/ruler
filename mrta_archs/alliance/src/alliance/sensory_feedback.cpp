#include "alliance/sensory_feedback.h"

namespace alliance
{
SensoryFeedback::SensoryFeedback(const TaskPtr &task) : task_(task) {}

SensoryFeedback::~SensoryFeedback()
{
}

bool SensoryFeedback::isApplicable(const ros::Time& timestamp) const
{
  std::list<SensorPtr>::const_iterator it(sensors_.begin());
  while (it != sensors_.end())
  {
    SensorPtr sensor = *it;
    if (!sensor->isUpToDate(timestamp))
    {
      return false;
    }
    it++;
  }
  return true;
}
}
