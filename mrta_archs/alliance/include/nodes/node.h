#ifndef _ALLIANCE__NODE_H_
#define _ALLIANCE__NODE_H_

#include "alliance/behaved_robot.h"
#include <utilities/ros_node.h>

namespace nodes
{
class Node : public utilities::ROSNode
{
public:
  Node(ros::NodeHandle* nh = new ros::NodeHandle(),
               float loop_rate = 30.0);
  virtual ~Node();

private:
  alliance::BehavedRobot* robot_;
  virtual void readParameters();
  virtual void controlLoop();
};
}

#endif // _ALLIANCE__NODE_H_
