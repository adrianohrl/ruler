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
    std::string id, std::string name, double capacity, double initial_level,
    ros::Duration latence)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          id, name, utilities::ContinuousSignalType(capacity),
          utilities::ContinuousSignalType(initial_level), latence)
{
}

ContinuousConsumableResource::ContinuousConsumableResource(
    std::string id, std::string name, utilities::ContinuousSignalType capacity,
    utilities::ContinuousSignalType initial_level, ros::Duration latence)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          id, name, capacity, initial_level, latence)
{
}

ContinuousConsumableResource::ContinuousConsumableResource(
    const ruler_msgs::Resource& msg)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          msg)
{
  utilities::SignalTypeEnum signal_type(
      utilities::SignalTypes::toEnumerated(msg.signal_type));
  if (signal_type != utilities::signal_types::CONTINUOUS)
  {
    throw utilities::Exception(
        "Not a continuous signal type resource ros message.");
  }
}

ContinuousConsumableResource::ContinuousConsumableResource(
    const ContinuousConsumableResource& resource)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          resource)
{
}

ContinuousConsumableResource::~ContinuousConsumableResource() {}
}
