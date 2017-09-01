#include "alliance_test/layer.h"

namespace alliance_test
{
Layer::Layer() : nh_(NULL), odometry_(NULL) {}

Layer::~Layer()
{
  velocity_pub_.shutdown();
  if (odometry_)
  {
    delete odometry_;
    odometry_ = NULL;
  }
  if (nh_)
  {
    delete nh_;
    nh_ = NULL;
  }
}

void Layer::initialize(const std::string& name)
{
  alliance::Layer::initialize(name);
  nh_ = new ros::NodeHandle();
  odometry_ = new nodes::ROSSensorMessage<nav_msgs::Odometry>(
      nh_, "odom", ros::Duration(1.0));
  velocity_pub_ = nh_->advertise<geometry_msgs::Twist>("cmd_vel", 10);
}

void Layer::process() { odometry_->publish(); }
}
