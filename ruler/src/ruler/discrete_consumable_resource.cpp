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
}

DiscreteConsumableResource::DiscreteConsumableResource(
    const DiscreteConsumableResource& resource)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          resource)
{
}

DiscreteConsumableResource::~DiscreteConsumableResource() {}
}
