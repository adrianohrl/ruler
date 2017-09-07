#include "alliance/sensory_feedback.h"

namespace alliance
{
SensoryFeedback::SensoryFeedback(const TaskPtr& task) : task_(task) {}

SensoryFeedback::~SensoryFeedback() {}

bool SensoryFeedback::isApplicable(const ros::Time& timestamp) const
{
  for (const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    SensorPtr sensor(*it);
    if (!sensor->isUpToDate(timestamp))
    {
      return false;
    }
  }
  return true;
}

void SensoryFeedback::addSensor(const SensorPtr& sensor)
{
  if (!contains(*sensor))
  {
    sensors_.push_back(sensor);
  }
}

std::size_t SensoryFeedback::size() const { return sensors_.size(); }

bool SensoryFeedback::empty() const { return sensors_.empty(); }

SensoryFeedback::iterator SensoryFeedback::begin() { return sensors_.begin(); }

SensoryFeedback::const_iterator SensoryFeedback::begin() const
{
  return sensors_.begin();
}

SensoryFeedback::iterator SensoryFeedback::end() { return sensors_.end(); }

SensoryFeedback::const_iterator SensoryFeedback::end() const
{
  return sensors_.end();
}

bool SensoryFeedback::contains(const Sensor& sensor) const
{
  for (const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    if (**it == sensor)
    {
      return true;
    }
  }
  return false;
}
}
