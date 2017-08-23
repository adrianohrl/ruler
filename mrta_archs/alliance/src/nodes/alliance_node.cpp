#include "nodes/alliance_node.h"

namespace nodes
{
AllianceNode::AllianceNode(ros::NodeHandle *nh, float loop_rate)
  : ROSNode::ROSNode(nh, loop_rate), robot_(NULL)
{

}

AllianceNode::~AllianceNode()
{
  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
}

void AllianceNode::readParameters()
{
  ros::NodeHandle pnh("~");
}

void AllianceNode::controlLoop()
{
}
}
