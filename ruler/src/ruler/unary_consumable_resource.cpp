/**
 *  This header file implements the UnaryConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_consumable_resource.h"

namespace ruler
{
UnaryConsumableResource::UnaryConsumableResource(std::string type,
                                                 std::string name,
                                                 bool initial_level,
                                                 ros::Duration latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          type, name, utilities::UnarySignalType(true),
          utilities::UnarySignalType(initial_level), latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    std::string type, std::string name,
    utilities::UnarySignalType initial_level, ros::Duration latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          type, name, utilities::UnarySignalType(true), initial_level, latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const UnaryConsumableResource& resource)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          resource)
{
}

UnaryConsumableResource::~UnaryConsumableResource() {}
}
