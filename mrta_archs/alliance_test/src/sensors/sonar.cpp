#include "sensors/sonar.h"

namespace sensors
{
Sonar::Sonar(const std::string& id, const ros::NodeHandlePtr& nh,
             const std::string& ns, const std::string& topic_name,
             const ros::Duration& timeout_duration)
    : ROSSensorMessage<sensor_msgs::PointCloud>::ROSSensorMessage(
          id + "/" + topic_name, nh, ns + "/" + topic_name, timeout_duration)
{
}

Sonar::~Sonar() {}

double Sonar::operator[](int index) const
{
  return index >= 0 && index < msg_.points.size()
             ? sqrt(pow(msg_.points[index].x, 2) + pow(msg_.points[index].y, 2))
             : 0.0;
}

double Sonar::getDistance(int index) const
{
  return index >= 0 && index < msg_.points.size()
             ? sqrt(pow(msg_.points[index].x, 2) + pow(msg_.points[index].y, 2))
             : 0.0;
}

geometry_msgs::Point32 Sonar::getPoint(int index) const
{
  return msg_.points[index];
}

bool Sonar::empty() const { return msg_.points.empty(); }

std::size_t Sonar::size() const { return msg_.points.size(); }

Sonar::iterator Sonar::begin() { return msg_.points.begin(); }

Sonar::const_iterator Sonar::begin() const { return msg_.points.begin(); }

Sonar::iterator Sonar::end() { return msg_.points.end(); }

Sonar::const_iterator Sonar::end() const { return msg_.points.end(); }
}
