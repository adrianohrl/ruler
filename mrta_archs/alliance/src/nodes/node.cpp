#include "nodes/node.h"

namespace nodes
{

Node::Node(ros::NodeHandle* nh, float loop_rate)
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
  ros::NodeHandle pnh("~/tasks");
  int size;
  pnh.param("size", size, 0);
  std::string id, name;
  std::list<alliance::Task> tasks;
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "task" << i << "/";
    pnh.param(ss.str() + "id", id, std::string(""));
    if (id.empty())
    {
      ROS_ERROR("The task's id must not be empty.");
      continue;
    }
    pnh.param(ss.str() + "name", name, std::string(""));
    alliance::Task task(id, name.empty() ? id : name);
    ss << "layers/";
    int layers_size;
    pnh.param("size", layers_size, 0);
    std::string plugin_name;
    for (int j(0); j < layers_size; j++)
    {
      std::stringstream sss;
      sss << ss.str() << "layer" << j << "/";
      pnh.param(ss.str() + "plugin_name", plugin_name, std::string(""));
      task.addNeededLayer(plugin_name);
    }
    tasks.push_back(task);
  }
  if (tasks.empty())
  {
    ROSNode::shutdown("Not found any task info as a ROS parameter.");
    return;
  }
  pnh = ros::NodeHandle("~");
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
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "behaviour_set" << i << "/";
    pnh.param(ss.str() + "task_id", id, std::string(""));
    if (id.empty())
    {
      ROS_ERROR("The behaviour set's task id must not be empty.");
      continue;
    }
    std::list<alliance::Task>::const_iterator it(tasks.begin());
    alliance::LayeredBehaviourSet* behaviour_set;
    while (it != tasks.end())
    {
      if (it->getId() == id)
      {
        behaviour_set =
            new alliance::LayeredBehaviourSet(robot_, new alliance::Task(*it));
        break;
      }
      it++;
    }
    robot_->addBehaviourSet(behaviour_set);
  }
  if (robot_->getBehaviourSets().empty())
  {
    ROSNode::shutdown("None behaviour set was imported to " + robot_->str() +
                      " robot.");
  }
}

void Node::controlLoop() { robot_->process(); }
}
