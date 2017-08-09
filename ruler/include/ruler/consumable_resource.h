/**
 *  This header file defines and implements the ConsumableResource abstract
 *class, which is based on the Resource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONSUMABLE_RESOURCE_H_
#define _RULER_CONSUMABLE_RESOURCE_H_

#include "utilities/functions/function.h"
#include "ruler/resource.h"

namespace ruler
{
template <typename T> class ConsumableResource : public Resource<T>
{
public:
  virtual ~ConsumableResource();
  virtual bool isConsumable() const;
  virtual void consume(Task* task,
                       utilities::functions::Function<T>* quantity_function);
  virtual void produce(Task* task,
                       utilities::functions::Function<T>* quantity_function);

protected:
  ConsumableResource(const ruler_msgs::Resource& msg);
  ConsumableResource(std::string id, std::string name, T capacity,
                     T initial_level,
                     ros::Duration latence = ros::Duration(0.0));
  ConsumableResource(const ConsumableResource<T>& resource);
};

template <typename T>
ConsumableResource<T>::ConsumableResource(const ruler_msgs::Resource& msg)
    : Resource<T>::Resource(msg)
{
  if (!msg.consumable)
  {
    throw utilities::Exception("Not a consumable resource ros message.");
  }
}

template <typename T>
ConsumableResource<T>::ConsumableResource(std::string id, std::string name,
                                          T capacity, T initial_level,
                                          ros::Duration latence)
    : Resource<T>::Resource(id, name, capacity, initial_level, latence)
{
}

template <typename T>
ConsumableResource<T>::ConsumableResource(const ConsumableResource<T>& resource)
    : Resource<T>::Resource(resource)
{
}

template <typename T> ConsumableResource<T>::~ConsumableResource() {}

template <typename T> bool ConsumableResource<T>::isConsumable() const
{
  return true;
}

template <typename T>
void ConsumableResource<T>::consume(
    Task* task, utilities::functions::Function<T>* quantity_function)
{
  if (!quantity_function->saturatesEnd())
  {
    throw utilities::Exception(
        "Unable to consume the " + Resource<T>::str() +
        " resource. The input quantity function does not " +
        " saturate after df.");
  }
  if (task->hasStarted())
  {
    throw utilities::Exception("Unable to consume the " + Resource<T>::str() +
                               " resource. The " + task->str() +
                               " task has already been started.");
  }
  quantity_function->setNegated(true);
  quantity_function->setAscending(true);
  Resource<T>::profile_->addTaskFunction(
      new TaskFunction<T>(this, task, quantity_function));
}

template <typename T>
void ConsumableResource<T>::produce(
    Task* task, utilities::functions::Function<T>* quantity_function)
{
  if (!quantity_function->saturatesEnd())
  {
    throw utilities::Exception(
        "Unable to produce the " + Resource<T>::str() +
        " resource. The input quantity function does not " +
        " saturate after df.");
  }
  if (task->hasStarted())
  {
    throw utilities::Exception("Unable to produce the " + Resource<T>::str() +
                               " resource. The " + task->str() +
                               " task has already been started.");
  }
  quantity_function->setNegated(false);
  quantity_function->setAscending(true);
  Resource<T>::profile_->addTaskFunction(
      new TaskFunction<T>(this, task, quantity_function));
}
}

#endif // _RULER_CONSUMABLE_RESOURCE_H_
