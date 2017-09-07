#include "alliance/sensor.h"

namespace alliance
{

Sensor::Sensor(const std::string& id) : HasId<std::string>::HasId(id) {}

Sensor::~Sensor() {}

bool Sensor::isUpToDate(const ros::Time& timestamp)
{
  return sample_holder_ && sample_holder_->getValue(timestamp);
}
}
