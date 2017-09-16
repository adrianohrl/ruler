/**
 *  This header file implements the DiscreteConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/discrete_consumable_resource.h"

namespace ruler
{
DiscreteConsumableResource::DiscreteConsumableResource(
    const std::string& id, const std::string& name, long capacity,
    long initial_level, const ros::Duration& latence)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          id, name, utilities::DiscreteSignalType(capacity),
          utilities::DiscreteSignalType(initial_level), latence)
{
}

DiscreteConsumableResource::DiscreteConsumableResource(
    const std::string& id, const std::string& name,
    const utilities::DiscreteSignalType& capacity,
    const utilities::DiscreteSignalType& initial_level,
    const ros::Duration& latence)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          id, name, capacity, initial_level, latence)
{
}

DiscreteConsumableResource::DiscreteConsumableResource(
    const ruler_msgs::Resource& msg)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(msg)
{
  utilities::SignalTypeEnum signal_type(
      utilities::SignalTypes::toEnumerated(msg.signal_type));
  if (signal_type != utilities::signal_types::DISCRETE)
  {
    throw utilities::Exception(
        "Not a discrete signal type resource ros message.");
  }
}

DiscreteConsumableResource::DiscreteConsumableResource(
    const DiscreteConsumableResource& resource)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          resource)
{
}

DiscreteConsumableResource::~DiscreteConsumableResource() {}
}
