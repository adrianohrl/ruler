/**
 *  This header file implements the UnaryConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_consumable_resource.h"
#include "utilities/functions/pulse_function.h"
#include "utilities/functions/step_function.h"

namespace ruler
{
UnaryConsumableResource::UnaryConsumableResource(
    const std::string& id, const std::string& name,
    const utilities::UnarySignalType& initial_level,
    const ros::Duration& latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          id, name, true, initial_level, latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const ruler_msgs::Resource& msg)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(msg)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const UnaryConsumableResource& resource)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          resource)
{
}

UnaryConsumableResource::~UnaryConsumableResource() {}

void UnaryConsumableResource::consume(const TaskPtr& task, double d0, double df)
{
  ConsumableResource<utilities::UnarySignalType>::consume(task, true, d0, df);
}

void UnaryConsumableResource::consume(const TaskPtr& task,
                                      const UnaryFunctionPtr& quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::consume(task,
                                                          quantity_function);
}

void UnaryConsumableResource::produce(const TaskPtr& task, double d0, double df)
{
  ConsumableResource<utilities::UnarySignalType>::produce(task, true, d0, df);
}

void UnaryConsumableResource::produce(const TaskPtr& task,
                                      const UnaryFunctionPtr& quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::produce(task,
                                                          quantity_function);
}
}
