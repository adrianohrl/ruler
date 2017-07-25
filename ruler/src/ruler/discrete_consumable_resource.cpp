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
DiscreteConsumableResource::DiscreteConsumableResource(std::string type,
                                                       std::string name,
                                                       long capacity,
                                                       long initial_level,
                                                       ros::Duration latence)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          type, name, utilities::DiscreteSignalType(capacity),
          utilities::DiscreteSignalType(initial_level), latence)
{
}

DiscreteConsumableResource::DiscreteConsumableResource(
    std::string type, std::string name, utilities::DiscreteSignalType capacity,
    utilities::DiscreteSignalType initial_level, ros::Duration latence)
    : ConsumableResource<utilities::DiscreteSignalType>::ConsumableResource(
          type, name, capacity, initial_level, latence)
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
