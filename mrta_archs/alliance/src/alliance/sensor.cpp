#include "alliance/sensor.h"

namespace alliance
{

Sensor::Sensor() {}

Sensor::~Sensor() {}

bool Sensor::isUpToDate(const ros::Time& timestamp)
{
  return sample_holder_ && sample_holder_->getValue(timestamp);
}
}
