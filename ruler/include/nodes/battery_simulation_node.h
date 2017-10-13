#ifndef _NODES_BATTERY_SIMULATION_NODE_H_
#define _NODES_BATTERY_SIMULATION_NODE_H_

#include <sensor_msgs/BatteryState.h>
#include <std_srvs/Trigger.h>
#include "ruler/ruler.h"
#include <ruler_msgs/DischargeBattery.h>
#include "utilities/ros_node.h"

namespace nodes
{
class BatterySimulationNode : public utilities::ROSNode
{
public:
  BatterySimulationNode(const ros::NodeHandlePtr& nh,
                        const ros::Rate& loop_rate = ros::Rate(30.0));
  virtual ~BatterySimulationNode();

private:
  std::string robot_id_;
  ros::Publisher battery_pub_;
  ros::ServiceServer recharge_srv_;
  ros::ServiceServer recharge_abort_srv_;
  ros::ServiceServer discharge_srv_;
  ros::ServiceServer discharge_abort_srv_;
  ruler::BatterySimulationPtr battery_;
  virtual void readParameters();
  virtual void controlLoop();
  virtual bool rechargeCallback(std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response);
  virtual bool rechargeAbortCallback(std_srvs::Trigger::Request& request,
                                     std_srvs::Trigger::Response& response);
  virtual bool
  dischargeCallback(ruler_msgs::DischargeBattery::Request& request,
                    ruler_msgs::DischargeBattery::Response& response);
  virtual bool
  dischargeAbortCallback(ruler_msgs::DischargeBattery::Request& request,
                         ruler_msgs::DischargeBattery::Response& response);
};

typedef boost::shared_ptr<BatterySimulationNode> BatterySimulationNodePtr;
}

#endif // _NODES_BATTERY_CHARGE_SIMULATION_NODE_H_
