#ifndef _ALLIANCE__NODE_H_
#define _ALLIANCE__NODE_H_

#include "alliance/behaved_robot.h"
#include "utilities/beacon_signal_subject.h"
#include <utilities/ros_node.h>

namespace nodes
{
class Node : public utilities::ROSNode, public utilities::BeaconSignalSubject
{
public:
  Node(ros::NodeHandlePtr nh, float loop_rate = 30.0);
  virtual ~Node();

private:
  ros::Subscriber beacon_signal_sub_;
  alliance::BehavedRobotPtr robot_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  void beaconSignalCallback(const alliance_msgs::BeaconSignal& msg);
};

typedef boost::scoped_ptr<Node> NodePtr;
}

#endif // _ALLIANCE__NODE_H_
