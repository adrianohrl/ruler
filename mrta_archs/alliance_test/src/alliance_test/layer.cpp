#include "alliance_test/layer.h"

namespace alliance_test
{
Layer::Layer() : nh_(new ros::NodeHandle()) {}

Layer::~Layer() { velocity_pub_.shutdown(); }

void Layer::initialize(const std::string& name)
{
  alliance::Layer::initialize(name);
  odometry_.reset(new nodes::ROSSensorMessage<nav_msgs::Odometry>(
      name + "/odom", nh_, "odom", ros::Duration(1.0)));
  velocity_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void Layer::process() { odometry_->publish(); }
}
