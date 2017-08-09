/**
 *  This source file implements the RulerTestNode class, which is based on the
 *ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/ruler_test_node.h"

namespace ruler_test
{

RulerTestNode::RulerTestNode(ros::NodeHandle* nh, float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate)
{
}

RulerTestNode::~RulerTestNode()
{
  resources_pub_.shutdown();
  for (int i(0); i < resource_pubs_.size(); i++)
  {
    resource_pubs_[i].shutdown();
  }
  for (int i(0); i < resources_.size(); i++)
  {
    if (resources_[i])
    {
      delete resources_[i];
      resources_[i] = NULL;
    }
  }
}

void RulerTestNode::readParameters()
{
  ros::NodeHandle pnh("~");
  int resources_size;
  pnh.param("resources/size", resources_size, 0);
  std::string type, signal_type, id, name;
  double latence, continuous_capacity, continuous_initial_level;
  int discrete_capacity, discrete_initial_level;
  bool unary_initial_level;
  ruler::ResourceInterface* resource;
  for (int i(0); i < resources_size; i++)
  {
    std::stringstream ss;
    ss << "resources/resource" << i << "/";
    pnh.param(ss.str() + "type", type, std::string(""));
    if (type.empty() || type != "consumable" && type != "reusable")
    {
      ROS_ERROR_STREAM("Invalid resource type: '" << type << "'.");
      continue;
    }
    pnh.param(ss.str() + "signal_type", signal_type, std::string(""));
    if (signal_type.empty() || !utilities::SignalTypes::isValid(signal_type))
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
    if (contains(*resource))
    {
      ROS_WARN_STREAM("Ignored already imported resource: " << *resource);
      continue;
    }
    resources_.push_back(resource);
    ROS_INFO_STREAM("Added resource: " << *resource);
  }
  if (resources_.empty())
  {
    ROS_FATAL("None resources were imported.");
    ROSNode::shutdown();
  }
}

void RulerTestNode::init()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  resources_pub_ =
      nh->advertise<ruler_msgs::Resource>("resources", resources_.size());
  for (int i(0); i < resources_.size(); i++)
  {
    resource_pubs_.push_back(nh->advertise<ruler_msgs::Resource>(
        "resources/" + resources_[i]->str(), 10));
  }
}

void RulerTestNode::controlLoop()
{
  if (resources_.empty())
  {
    ROS_FATAL("None resources were imported.");
    ROSNode::shutdown();
  }
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  for (int i(0); i < resources_.size(); i++)
  {
    ruler_msgs::Resource msg(resources_[i]->toMsg());
    resources_pub_.publish(msg);
    resource_pubs_[i].publish(msg);
  }
}

bool RulerTestNode::contains(const ruler::ResourceInterface& resource) const
{
  for (int i(0); i < resources_.size(); i++)
  {
    if (*resources_[i] == resource)
    {
      return true;
    }
  }
  return false;
}
}
