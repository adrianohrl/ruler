#ifndef _ALLIANCE_LOW_LEVEL_NODE_H_
#define _ALLIANCE_LOW_LEVEL_NODE_H_

#include "alliance/behaved_robot.h"
#include "utilities/beacon_signal_subject.h"
#include <utilities/ros_node.h>

namespace nodes
{
class LowLevelNode : public utilities::ROSNode,
                     public utilities::BeaconSignalSubject
{
public:
  LowLevelNode(const ros::NodeHandlePtr& nh,
               const ros::Rate& rate = ros::Rate(30.0));
  virtual ~LowLevelNode();

private:
  ros::Subscriber beacon_signal_sub_;
  alliance::BehavedRobotPtr robot_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  void beaconSignalCallback(const alliance_msgs::BeaconSignal& msg);
};

typedef boost::scoped_ptr<LowLevelNode> LowLevelNodePtr;
}

#endif // _ALLIANCE_LOW_LEVEL_NODE_H_
