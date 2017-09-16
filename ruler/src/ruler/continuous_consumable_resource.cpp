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
    const std::string& id, const std::string& name, double capacity,
    double initial_level, const ros::Duration& latence)
    : ConsumableResource<utilities::ContinuousSignalType>::ConsumableResource(
          id, name, utilities::ContinuousSignalType(capacity),
          utilities::ContinuousSignalType(initial_level), latence)
{
}

ContinuousConsumableResource::ContinuousConsumableResource(
    const std::string& id, const std::string& name,
    const utilities::ContinuousSignalType& capacity,
    const utilities::ContinuousSignalType& initial_level,
    const ros::Duration& latence)
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
