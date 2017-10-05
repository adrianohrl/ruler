#include "alliance/sensor.h"

namespace alliance
{
Sensor::Sensor(const std::string& id)
    : HasId<std::string>::HasId(id) /*,
       applicable_(new SampleHolder(id, buffer_horizon))*/
{
}

Sensor::~Sensor() {}

bool Sensor::isApplicable(const ros::Time& timestamp)
{
  return applicable_->getValue(timestamp);
}

void Sensor::update(const ros::Time& timestamp)
{
  applicable_->update(timestamp);
}
}
