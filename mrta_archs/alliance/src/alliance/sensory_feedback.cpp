#include "alliance/sensory_feedback.h"

namespace alliance
{
SensoryFeedback::SensoryFeedback(Task* task) : task_(task) {}

SensoryFeedback::~SensoryFeedback()
{
  std::list<Sensor*>::iterator it(sensors_.begin());
  while (it != sensors_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
  task_ = NULL;
}

bool SensoryFeedback::isApplicable(const ros::Time& timestamp) const
{
  std::list<Sensor*>::const_iterator it(sensors_.begin());
  while (it != sensors_.end())
  {
    Sensor* sensor = *it;
    if (!sensor->isUpToDate(timestamp))
    {
      return false;
    }
    it++;
  }
  return true;
}
}
