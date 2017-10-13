#include "alliance_test/layer.h"

namespace alliance_test
{
Layer::Layer() : nh_(new ros::NodeHandle()) {}

Layer::~Layer() { velocity_pub_.shutdown(); }

void Layer::initialize(const std::string& ns, const std::string& name)
{
  alliance::Layer::initialize(ns, name);
  //odometry_.reset(new Odometry(name, nh_, ns));
  //sonars_.reset(new PointCloud(name, nh_, ns));
  velocity_pub_ = nh_->advertise<geometry_msgs::Twist>(ns + "/cmd_vel", 10);
}

void Layer::process()
{
  velocity_pub_.publish(velocity_msg_);
}

void Layer::setVelocity(double vx, double wz)
{
  velocity_msg_.linear.x = vx;
  velocity_msg_.angular.z = wz;
}
}
