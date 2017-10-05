#include "nodes/battery_simulation_node.h"

namespace nodes
{
BatterySimulationNode::BatterySimulationNode(const ros::NodeHandlePtr& nh,
                                             const ros::Rate& loop_rate)
    : ROSNode::ROSNode(nh, loop_rate)
{
  battery_pub_ = nh->advertise<sensor_msgs::BatteryState>("battery_state", 1);
  recharge_srv_ = nh->advertiseService(
      "recharge", &BatterySimulationNode::rechargeCallback, this);
}

BatterySimulationNode::~BatterySimulationNode()
{
  battery_pub_.shutdown();
  recharge_srv_.shutdown();
}

void BatterySimulationNode::readParameters()
{
  ROSNode::readParameters();
  ros::NodeHandle pnh("~");
  pnh.param("id", robot_id_, std::string(""));
  if (robot_id_.empty())
  {
    ROSNode::shutdown("Not found robot id as a ROS parameter.");
    return;
  }
  if (!std::equal(robot_id_.rbegin(), robot_id_.rend(),
                  ros::this_node::getNamespace().rbegin()))
  {
    ROSNode::shutdown("Invalid ROS namespace. It must end with '" + robot_id_ +
                      "'.");
    return;
  }
  double sample_time, sample_time_std;
  pnh.param("expected_sample_time/mean", sample_time, 0.0);
  pnh.param("expected_sample_time/standard_deviation", sample_time_std, 0.0);
  utilities::ContinuousNoisySignalPtr expected_sample_time(
      new utilities::ContinuousNoisySignal(sample_time, sample_time_std));
  pnh = ros::NodeHandle("~/battery");
  double slow_discharging_rate;
  pnh.param("slow_discharging_rate", slow_discharging_rate, 0.001);
  if (slow_discharging_rate <= 0.0 || slow_discharging_rate >= 1.0)
  {
    ROS_WARN("The battery slow discharging rate must be within the (0.0; 1.0) "
             "interval.");
    slow_discharging_rate = 0.001;
  }
  double low_threshold;
  pnh.param("low_threshold", low_threshold, 0.15);
  if (low_threshold <= 0.0 || low_threshold >= 1.0)
  {
    ROS_WARN(
        "The battery low threshold must be within the (0.0; 1.0) interval.");
    low_threshold = 0.15;
  }
  double critical_threshold;
  pnh.param("critical_threshold", critical_threshold, 0.05);
  if (critical_threshold <= 0.0 || critical_threshold >= 1.0)
  {
    ROS_WARN("The battery critical threshold must be within the (0.0; 1.0) "
             "interval.");
    critical_threshold = 0.05;
  }
  double low_warning_rate;
  pnh.param("low_warning_rate", low_warning_rate, 0.25);
  if (low_warning_rate <= 0.0)
  {
    ROS_WARN("The battery low level warning rate must be positive.");
    low_warning_rate = 0.25;
  }
  double critical_warning_rate;
  pnh.param("critical_warning_rate", critical_warning_rate, 0.15);
  if (critical_warning_rate <= 0.0)
  {
    ROS_WARN("The battery critical level warning rate must be positive.");
    critical_warning_rate = 1.0;
  }
  battery_.reset(new ruler::BatterySimulation(
      robot_id_, expected_sample_time, slow_discharging_rate, low_threshold,
      critical_threshold, ros::Rate(low_warning_rate),
      ros::Rate(critical_warning_rate)));
}

void BatterySimulationNode::controlLoop()
{
  sensor_msgs::BatteryState msg;
  msg.header.frame_id = robot_id_;
  msg.header.stamp = ros::Time::now();
  battery_->update(msg.header.stamp);
  msg.percentage = battery_->getRemainingCharge();
  battery_pub_.publish(msg);
}

bool BatterySimulationNode::rechargeCallback(
    std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  battery_->recharge();
  response.success = !battery_->isFull();
}
}
