#ifndef _ALLIANCE_NODE_H_
#define _ALLIANCE_NODE_H_

#include "alliance/robot.h"
#include <utilities/ros_node.h>

namespace nodes
{
class AllianceNode : public utilities::ROSNode
{
public:
  AllianceNode(ros::NodeHandle *nh = new ros::NodeHandle(), float loop_rate = 30.0);
  virtual ~AllianceNode();

private:
  alliance::Robot* robot_;
  virtual void readParameters();
  virtual void controlLoop();
};
}

#endif // _ALLIANCE_NODE_H_
