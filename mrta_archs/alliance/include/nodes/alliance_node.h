#ifndef _ALLIANCE_NODE_H_
#define _ALLIANCE_NODE_H_

#include <alliance_msgs/BeaconSignal.h>
#include "utilities/beacon_signal_subject.h"
#include <utilities/ros_node.h>
#include "alliance/robot.h"

namespace nodes
{
class AllianceNode : public utilities::ROSNode,
                     public utilities::BeaconSignalSubject
{
public:
  AllianceNode(ros::NodeHandle* nh = new ros::NodeHandle(),
               float loop_rate = 30.0);
  virtual ~AllianceNode();

private:
  alliance::Robot* robot_;
  ros::Publisher beacon_signal_pub_;
  ros::Subscriber beacon_signal_sub_;
  virtual void readParameters();
  virtual void controlLoop();
  void beaconSignalCallback(const alliance_msgs::BeaconSignal& msg);
};
}

#endif // _ALLIANCE_NODE_H_
