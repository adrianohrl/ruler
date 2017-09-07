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
  typedef std::list<SensorPtr>::iterator iterator;
  typedef std::list<SensorPtr>::const_iterator const_iterator;
  SensoryFeedback(const TaskPtr& task);
  virtual ~SensoryFeedback();
  bool isApplicable(const ros::Time& timestamp = ros::Time::now()) const;
  void addSensor(const SensorPtr& sensor);
  std::size_t size() const;
  bool empty() const;
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

private:
  const TaskPtr task_;
  std::list<SensorPtr> sensors_;
  bool contains(const Sensor& sensor) const;
};

typedef boost::shared_ptr<SensoryFeedback> SensoryFeedbackPtr;
typedef boost::shared_ptr<SensoryFeedback const> SensoryFeedbackConstPtr;
}

#endif // _ALLIANCE_SENSORY_FEEDBACK_H_
