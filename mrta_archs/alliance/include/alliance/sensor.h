#ifndef _ALLIANCE_SENSOR_H_
#define _ALLIANCE_SENSOR_H_

#include <ros/time.h>
#include <utilities/functions/unary_sample_holder.h>
#include <utilities/has_id.h>
//#include "utilities/sensory_feedback_observer.h"

namespace alliance
{
class Sensor : public utilities::HasId<std::string>/*,
               public utilities::SensoryFeedbackObserver*/
{
public:
  Sensor(const std::string& id);
  virtual ~Sensor();
  virtual bool isApplicable(const ros::Time& timestamp = ros::Time::now());
  void update(const ros::Time& timestamp = ros::Time::now());

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  SampleHolderPtr applicable_;
};

typedef boost::shared_ptr<Sensor> SensorPtr;
typedef boost::shared_ptr<Sensor const> SensorConstPtr;
}

#endif // _ALLIANCE_SENSOR_H_
