/**
 *  This source file implements the RulerNode class, which is based on the
 *ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/ruler_node.h"

namespace nodes
{
RulerNode::RulerNode(ros::NodeHandle* nh, float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate), robot_(NULL)
{
}

RulerNode::~RulerNode()
{
  resources_pub_.shutdown();
  std::map<std::string, ros::Publisher>::iterator pub_it(resource_pubs_.begin());
  while (pub_it != resource_pubs_.end())
  {
    ros::Publisher pub(pub_it->second);
    pub.shutdown();
    pub_it++;
  }
  std::list<MetricsServiceServer*>::iterator srv_it(metrics_srvs_.begin());
  while (srv_it != metrics_srvs_.end())
  {
    if (*srv_it)
    {
      delete *srv_it;
      *srv_it = NULL;
    }
    srv_it++;
  }
  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
}

void RulerNode::readParameters()
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
  robot_ = new ruler::Robot(id, name);
  pnh = ros::NodeHandle("~/metrics");
  int size;
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "metric" << i << "/";
    pnh.param(ss.str() + "name", name, std::string(""));
    if (name.empty())
    {
      ROS_ERROR("The service server name must not be empty.");
      continue;
    }
    MetricsServiceServer *server = new MetricsServiceServer(getNodeHandle(), name);
    server->readPlugins(robot_, pnh.getNamespace() + "/" + ss.str());
    metrics_srvs_.push_back(server);
  }
  if (metrics_srvs_.empty())
  {
    ROSNode::shutdown("None metrics service server was imported.");
    return;
  }
  pnh = ros::NodeHandle("~/resources");
  pnh.param("size", size, 0);
  std::string type, signal_type;
  double latence, continuous_capacity, continuous_initial_level;
  int discrete_capacity, discrete_initial_level;
  bool unary_initial_level;
  ruler::ResourceInterface* resource;
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "resource" << i << "/";
    pnh.param(ss.str() + "type", type, std::string(""));
    if (type != "consumable" && type != "reusable")
    {
      ROS_ERROR_STREAM("Invalid resource type: '" << type << "'.");
      continue;
    }
    pnh.param(ss.str() + "signal_type", signal_type, std::string(""));
    if (!utilities::SignalTypes::isValid(signal_type))
    {
      ROS_ERROR_STREAM("Invalid resource signal type: '" << signal_type
                                                         << "'.");
      continue;
    }
    pnh.param(ss.str() + "id", id, std::string(""));
    pnh.param(ss.str() + "name", name, std::string(""));
    pnh.param(ss.str() + "latence", latence, 0.0);
    try
    {
      switch (utilities::SignalTypes::toEnumerated(signal_type))
      {
      case utilities::signal_types::CONTINUOUS:
        pnh.param(ss.str() + "capacity", continuous_capacity, -1.0);
        pnh.param(ss.str() + "initial_level", continuous_initial_level, 0.0);
        if (type == "consumable")
        {
          resource = new ruler::ContinuousConsumableResource(
              id, name, continuous_capacity, continuous_initial_level,
              ros::Duration(latence));
        }
        else
        {
          resource = new ruler::ContinuousReusableResource(
              id, name, continuous_capacity, continuous_initial_level,
              ros::Duration(latence));
        }
        break;
      case utilities::signal_types::DISCRETE:
        pnh.param(ss.str() + "capacity", discrete_capacity, -1);
        pnh.param(ss.str() + "initial_level", discrete_initial_level, 0);
        if (type == "consumable")
        {
          resource = new ruler::DiscreteConsumableResource(
              id, name, discrete_capacity, discrete_initial_level,
              ros::Duration(latence));
        }
        else
        {
          resource = new ruler::DiscreteReusableResource(
              id, name, discrete_capacity, discrete_initial_level,
              ros::Duration(latence));
        }
        break;
      case utilities::signal_types::UNARY:
        pnh.param(ss.str() + "initial_level", unary_initial_level, false);
        if (type == "consumable")
        {
          resource = new ruler::UnaryConsumableResource(
              id, name, unary_initial_level, ros::Duration(latence));
        }
        else
        {
          resource = new ruler::UnaryReusableResource(
              id, name, unary_initial_level, ros::Duration(latence));
        }
      }
    }
    catch (utilities::Exception e)
    {
      continue;
    }
    if (robot_->contains(*resource))
    {
      ROS_WARN_STREAM("Ignored already imported resource: " << *resource);
      continue;
    }
    robot_->addResource(resource);
    ROS_INFO_STREAM("   Added resource: " << *resource << " to " << *robot_
                                          << ".");
  }
  if (robot_->getResources().empty())
  {
    ROSNode::shutdown("None resource was imported.");
  }
}

void RulerNode::init()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  std::list<ruler::ResourceInterface*> resources(robot_->getResources());
  resources_pub_ =
      nh->advertise<ruler_msgs::Resource>("resources", resources.size());
  std::list<ruler::ResourceInterface*>::const_iterator it(resources.begin());
  while (it != resources.end())
  {
    ruler::ResourceInterface* resource = *it;
    resource_pubs_.insert(std::pair<std::string, ros::Publisher>(
        resource->getId(), nh->advertise<ruler_msgs::Resource>(
                               "resources/" + resource->str(), 10)));
    it++;
  }
}

void RulerNode::controlLoop()
{
  std::list<ruler::ResourceInterface*> resources(robot_->getResources());
  std::list<ruler::ResourceInterface*>::const_iterator it(resources.begin());
  while (it != resources.end())
  {
    ruler::ResourceInterface* resource = *it;
    ruler_msgs::Resource msg(resource->toMsg());
    resources_pub_.publish(msg);
    resource_pubs_.at(resource->getId()).publish(msg);
    it++;
  }
}
}
