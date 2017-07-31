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
  virtual void consume(Task* task, utilities::functions::Function<T>* quantity_function);
  virtual void produce(Task* task, utilities::functions::Function<T>* quantity_function);

protected:
  ConsumableResource(std::string id, std::string name, T capacity,
                     T initial_level,
                     ros::Duration latence = ros::Duration(0.0));
  ConsumableResource(const ConsumableResource<T>& resource);
};

template <typename T>
ConsumableResource<T>::ConsumableResource(std::string id, std::string name,
                                          T capacity, T initial_level,
                                          ros::Duration latence)
    : Resource<T>::Resource(id, true, name, capacity, initial_level, latence)
{
}

template <typename T>
ConsumableResource<T>::ConsumableResource(const ConsumableResource<T>& resource)
    : Resource<T>::Resource(resource)
{
}

template <typename T> ConsumableResource<T>::~ConsumableResource() {}

template <typename T>
void ConsumableResource<T>::consume(Task* task,
                                    utilities::functions::Function<T>* quantity_function)
{
  quantity_function->setAscending(false);
  Resource<T>::profile_->addTaskFunction(
      new TaskFunction<T>(task, quantity_function));
}

template <typename T>
void ConsumableResource<T>::produce(Task* task,
                                    utilities::functions::Function<T>* quantity_function)
{
  quantity_function->setAscending(true);
  Resource<T>::profile_->addTaskFunction(
      new TaskFunction<T>(task, quantity_function));
}
}

#endif // _RULER_CONSUMABLE_RESOURCE_H_
