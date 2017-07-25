/**
 *  This header file implements the ContinuousConsumableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/continuous_consumable_resource.h"

namespace ruler
{
ContinuousConsumableResource::ContinuousConsumableResource(
    std::string type, std::string name, double capacity, double initial_level,
    ros::Duration latence)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          type, name, utilities::ContinuousSignalType(capacity),
          utilities::ContinuousSignalType(initial_level), latence)
{
}

ContinuousConsumableResource::ContinuousConsumableResource(
    std::string type, std::string name,
    utilities::ContinuousSignalType capacity,
    utilities::ContinuousSignalType initial_level, ros::Duration latence)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          type, name, capacity, initial_level, latence)
{
}

ContinuousConsumableResource::ContinuousConsumableResource(
    const ContinuousConsumableResource& resource)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          resource)
{
}

ContinuousConsumableResource::~ContinuousConsumableResource() {}
}
