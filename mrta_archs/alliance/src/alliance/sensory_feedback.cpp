#include "alliance/sensory_feedback.h"

namespace alliance
{
SensoryFeedback::SensoryFeedback(Task* task) : task_(task) {}

SensoryFeedback::SensoryFeedback(const SensoryFeedback& sensory_feedback)
  : task_(sensory_feedback.task_)
{
  std::list<Sensor*>::const_iterator it(sensory_feedback.sensors_.begin());
  while (it != sensory_feedback.sensors_.end())
  {
    sensors_.push_back(new Sensor(**it));
    it++;
  }
}

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
