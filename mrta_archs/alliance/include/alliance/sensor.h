#ifndef _ALLIANCE_SENSOR_H_
#define _ALLIANCE_SENSOR_H_

#include <ros/time.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
class Sensor
{
public:
  Sensor();
  virtual ~Sensor();
  bool isUpToDate(const ros::Time timestamp = ros::Time::now());

private:
  utilities::functions::UnarySampleHolder* sample_holder_;
};
}

#endif // _ALLIANCE_SENSOR_H_
