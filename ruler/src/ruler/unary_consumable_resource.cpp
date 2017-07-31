/**
 *  This header file implements the UnaryConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_consumable_resource.h"
#include "utilities/functions/step_function.h"

namespace ruler
{
UnaryConsumableResource::UnaryConsumableResource(std::string id,
                                                 std::string name,
                                                 bool initial_level,
                                                 ros::Duration latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          id, name, utilities::UnarySignalType(true),
          utilities::UnarySignalType(initial_level), latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    std::string id, std::string name, utilities::UnarySignalType initial_level,
    ros::Duration latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          id, name, utilities::UnarySignalType(true), initial_level, latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const UnaryConsumableResource& resource)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          resource)
{
}

UnaryConsumableResource::~UnaryConsumableResource() {}

void UnaryConsumableResource::consume(Task* task)
{
  /*ConsumableResource<utilities::UnarySignalType>::consume(
      task, new utilities::StepFunction());*/
}

void UnaryConsumableResource::produce(Task* task)
{
  /*ConsumableResource<utilities::UnarySignalType>::produce(
      task, new utilities::StepFunction());*/
}
}
