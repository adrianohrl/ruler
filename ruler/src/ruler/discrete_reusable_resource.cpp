/**
 *  This header file implements the DiscreteReusableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/discrete_reusable_resource.h"

namespace ruler
{
DiscreteReusableResource::DiscreteReusableResource(const std::string& id,
                                                   const std::string& name,
                                                   long capacity,
                                                   long initial_level,
                                                   const ros::Duration& latence)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          id, name, utilities::DiscreteSignalType(capacity),
          utilities::DiscreteSignalType(initial_level), latence)
{
}

DiscreteReusableResource::DiscreteReusableResource(
    const std::string& id, const std::string& name,
    const utilities::DiscreteSignalType& capacity,
    const utilities::DiscreteSignalType& initial_level,
    const ros::Duration& latence)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          id, name, capacity, initial_level, latence)
{
}

DiscreteReusableResource::DiscreteReusableResource(
    const ruler_msgs::Resource& msg)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(msg)
{
  utilities::SignalTypeEnum signal_type(
      utilities::SignalTypes::toEnumerated(msg.signal_type));
  if (signal_type != utilities::signal_types::DISCRETE)
  {
    throw utilities::Exception(
        "Not a discrete signal type resource ros message.");
  }
}

DiscreteReusableResource::DiscreteReusableResource(
    const DiscreteReusableResource& resource)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          resource)
{
}

DiscreteReusableResource::~DiscreteReusableResource() {}

void DiscreteReusableResource::require(const TaskPtr& task, long quantity,
                                       double d0)
{
  ReusableResource<utilities::DiscreteSignalType>::require(
      task, utilities::DiscreteSignalType(quantity), d0);
}
}
