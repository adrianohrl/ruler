#include "alliance/sensor.h"

namespace alliance
{

Sensor::Sensor() : sample_holder_(NULL) {}

Sensor::~Sensor()
{
  if (sample_holder_)
  {
    delete sample_holder_;
    sample_holder_ = NULL;
  }
}

bool Sensor::isUpToDate(const ros::Time timestamp)
{
  return sample_holder_ && sample_holder_->getValue(timestamp);
}
}
