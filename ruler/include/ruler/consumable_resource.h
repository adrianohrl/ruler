/**
 *  This header file defines and implements the ConsumableResource abstract
 *class, which is based on the Resource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONSUMABLE_RESOURCE_H_
#define _RULER_CONSUMABLE_RESOURCE_H_

#include "utilities/function.h"
#include "ruler/resource.h"

namespace ruler
{
template <typename T> class ConsumableResource : public Resource<T>
{
public:
  virtual ~ConsumableResource();
  void consume(Task* task, utilities::Function* quantity);
  void produce(Task* task, utilities::Function* quantity);

protected:
  ConsumableResource(const ConsumableResource<T>& resource);
  ConsumableResource(std::string type, std::string name, T capacity,
                     T initial_level,
                     ros::Duration latence = ros::Duration(0.0));
};

template <typename T>
ConsumableResource<T>::ConsumableResource(std::string type, std::string name,
                                          T capacity, T initial_level,
                                          ros::Duration latence)
    : Resource<T>::Resource(type, name, capacity, initial_level, latence)
{
}

template <typename T>
ConsumableResource<T>::ConsumableResource(const ConsumableResource<T>& resource)
    : Resource<T>::Resource(resource)
{
}

template <typename T> ConsumableResource<T>::~ConsumableResource() {}

template <typename T>
void ConsumableResource<T>::consume(Task* task, utilities::Function* quantity)
{
}

template <typename T>
void ConsumableResource<T>::produce(Task* task, utilities::Function* quantity)
{
}
}

#endif // _RULER_CONSUMABLE_RESOURCE_H_
