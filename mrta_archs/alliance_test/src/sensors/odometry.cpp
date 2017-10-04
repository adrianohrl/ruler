#include "sensors/odometry.h"
#include <tf/transform_datatypes.h>

namespace sensors
{
Odometry::Odometry(const std::string& id, const ros::NodeHandlePtr& nh,
                   const std::string& ns, const std::string& topic_name,
                   const ros::Duration& timeout_duration)
    : ROSSensorMessage<nav_msgs::Odometry>::ROSSensorMessage(
          id + "/" + topic_name, nh, ns + "/" + topic_name, timeout_duration)
{
}

Odometry::~Odometry() {}

double Odometry::getX() const { return msg_.pose.pose.position.x; }

double Odometry::getY() const { return msg_.pose.pose.position.y; }

double Odometry::getYaw() const
{
  double yaw(tf::getYaw(msg_.pose.pose.orientation));
  while (yaw <= -M_PI || yaw > M_PI)
  {
    yaw += (yaw > M_PI ? -1 : 1) * 2 * M_PI;
  }
  return yaw;
}
}
