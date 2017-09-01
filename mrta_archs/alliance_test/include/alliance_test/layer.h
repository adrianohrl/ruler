#ifndef _ALLIANCE_TEST_LAYER_H_
#define _ALLIANCE_TEST_LAYER_H_

#include <alliance/layer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nodes/ros_sensor_message.h>

namespace alliance_test
{
class Layer : public alliance::Layer
{
public:
  Layer();
  virtual ~Layer();
  virtual void initialize(const std::string& name);
  virtual void process();

protected:
  ros::NodeHandle* nh_;
  ros::Publisher velocity_pub_;
  nodes::ROSSensorMessage<nav_msgs::Odometry>* odometry_;
};
}

#endif // _ALLIANCE_TEST_LAYER_H_
