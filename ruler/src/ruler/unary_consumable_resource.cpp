/**
 *  This header file implements the UnaryConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_consumable_resource.h"
#include "utilities/functions/unary_pulse_function.h"
#include "utilities/functions/unary_step_function.h"

namespace ruler
{
UnaryConsumableResource::UnaryConsumableResource(const std::string& id,
                                                 const std::string& name,
                                                 bool initial_level,
                                                 const ros::Duration& latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          id, name, utilities::UnarySignalType(true),
          utilities::UnarySignalType(initial_level), latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const std::string& id, const std::string& name,
    const utilities::UnarySignalType& initial_level,
    const ros::Duration& latence)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(
          id, name, utilities::UnarySignalType(true), initial_level, latence)
{
}

UnaryConsumableResource::UnaryConsumableResource(
    const ruler_msgs::Resource& msg)
    : ConsumableResource<utilities::UnarySignalType>::ConsumableResource(msg)
{
  utilities::SignalTypeEnum signal_type(
      utilities::SignalTypes::toEnumerated(msg.signal_type));
  if (signal_type != utilities::signal_types::UNARY)
  {
    throw utilities::Exception(
        "Not an unary signal type resource ros message.");
  }
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
  UnaryFunctionPtr quantity_function;
  if (df == INFINITY)
  {
    quantity_function.reset(new utilities::functions::UnaryStepFunction(d0));
  }
  else
  {
    quantity_function.reset(
        new utilities::functions::UnaryPulseFunction(d0, df));
  }
  consume(task, quantity_function);
}

void UnaryConsumableResource::consume(const TaskPtr& task,
                                      const UnaryFunctionPtr& quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::consume(task,
                                                          quantity_function);
}

void UnaryConsumableResource::produce(const TaskPtr& task, double d0, double df)
{
  UnaryFunctionPtr quantity_function;
  if (df == INFINITY)
  {
    quantity_function.reset(new utilities::functions::UnaryStepFunction(d0));
  }
  else
  {
    quantity_function.reset(
        new utilities::functions::UnaryPulseFunction(d0, df));
  }
  produce(task, quantity_function);
}

void UnaryConsumableResource::produce(const TaskPtr& task,
                                      const UnaryFunctionPtr& quantity_function)
{
  ConsumableResource<utilities::UnarySignalType>::produce(task,
                                                          quantity_function);
}
}
