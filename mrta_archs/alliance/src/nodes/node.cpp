#include "nodes/node.h"

namespace nodes
{

Node::Node(ros::NodeHandle *nh, float loop_rate)
  : ROSNode::ROSNode(nh, loop_rate), robot_(NULL)
{

}

Node::~Node()
{
  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
}

void Node::readParameters()
{
  ros::NodeHandle pnh("~");
  std::string id, name;
  pnh.param("id", id, std::string(""));
  pnh.param("name", name, std::string(""));
  if (id.empty())
  {
    ROSNode::shutdown("Not found robot id as a ROS parameter.");
    return;
  }
  if (!std::equal(id.rbegin(), id.rend(),
                  ros::this_node::getNamespace().rbegin()))
  {
    ROSNode::shutdown("Invalid ROS namespace. It must end with '" + id + "'.");
    return;
  }
  robot_ = new alliance::BehavedRobot(id, name);
}

void Node::controlLoop()
{
  robot_->process();
}

}
