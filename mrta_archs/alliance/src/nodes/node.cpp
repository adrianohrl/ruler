#include "nodes/node.h"

namespace nodes
{

Node::Node(ros::NodeHandle* nh, float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate),
      BeaconSignalSubject::BeaconSignalSubject(ros::this_node::getName()),
      robot_(NULL)
{
  beacon_signal_sub_ = nh->subscribe("/alliance/beacon_signal", 100,
                                     &Node::beaconSignalCallback, this);
}

Node::~Node()
{
  beacon_signal_sub_.shutdown();
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
    pnh.param(ss.str() + "size", layers_size, 0);
    std::string plugin_name;
    for (int j(0); j < layers_size; j++)
    {
      std::stringstream sss;
      sss << ss.str() << "layer" << j << "/";
      pnh.param(sss.str() + "plugin_name", plugin_name, std::string(""));
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
  pnh = ros::NodeHandle("~/behaviour_sets");
  pnh.param("size", size, 0);
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
    double buffer_horizon, timeout_duration;
    pnh.param("buffer_horizon", buffer_horizon, 5.0);
    if (buffer_horizon <= 0.0)
    {
      ROS_WARN(
          "The active buffer horizon of the behaviour set must be positive.");
      buffer_horizon = 5.0;
    }
    pnh.param("timeout_duration", timeout_duration, 2.0);
    if (timeout_duration < 0.0)
    {
      ROS_WARN("The active timeout duration of the behaviour set must not be negative.");
      timeout_duration = 2.0;
    }
    std::list<alliance::Task>::const_iterator it(tasks.begin());
    alliance::LayeredBehaviourSet* behaviour_set;
    while (it != tasks.end())
    {
      if (it->getId() == id)
      {
        behaviour_set = new alliance::LayeredBehaviourSet(
            robot_, new alliance::Task(*it), ros::Duration(buffer_horizon),
            ros::Duration(timeout_duration));
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

void Node::init()
{
  /** registering beacon signal message observers **/
  std::list<alliance::LayeredBehaviourSet*> behaviour_sets(
      robot_->getBehaviourSets());
  std::list<alliance::LayeredBehaviourSet*>::iterator it(
      behaviour_sets.begin());
  while (it != behaviour_sets.end())
  {
    BeaconSignalSubject::registerObserver(*it);
    it++;
  }
}

void Node::controlLoop() { robot_->process(); }

void Node::beaconSignalCallback(const alliance_msgs::BeaconSignal& msg)
{
  BeaconSignalSubject::notify(msg);
}
}
