/**
 *  This header file implements the UnaryConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_consumable_resource.h"
#include "utilities/functions/unary_step_function.h"
#include "utilities/functions/unary_pulse_function.h"

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

void UnaryConsumableResource::consume(Task* task, double d0, double df)
{
  utilities::functions::Function<utilities::UnarySignalType>* quantity_function;
  if (df == INFINITY)
  {
    quantity_function = new utilities::functions::UnaryStepFunction(d0);
  }
  else
  {
    quantity_function = new utilities::functions::UnaryPulseFunction(d0, df);
  }
  consume(task, quantity_function);
}

void UnaryConsumableResource::consume(
    Task* task, utilities::functions::Function<utilities::UnarySignalType>*
                    quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::consume(task,
                                                          quantity_function);
}

void UnaryConsumableResource::produce(Task* task, double d0, double df)
{
  utilities::functions::Function<utilities::UnarySignalType>* quantity_function;
  if (df == INFINITY)
  {
    quantity_function = new utilities::functions::UnaryStepFunction(d0);
  }
  else
  {
    quantity_function = new utilities::functions::UnaryPulseFunction(d0, df);
  }
  produce(task, quantity_function);
}

void UnaryConsumableResource::produce(
    Task* task, utilities::functions::Function<utilities::UnarySignalType>*
                    quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::produce(task,
                                                          quantity_function);
}
}
