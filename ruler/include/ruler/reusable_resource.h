/**
 *  This header file defines and implements the ReusableResource abstract class,
 *which is based on the Resource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_REUSABLE_RESOURCE_H_
#define _RULER_REUSABLE_RESOURCE_H_

#include "ruler/resource.h"
#include "utilities/functions/step_function.h"
#include "utilities/functions/pulse_function.h"

namespace ruler
{
template <typename T> class ReusableResource : public Resource<T>
{
public:
  virtual ~ReusableResource();
  virtual void require(Task* task, T quantity, double d0 = 0.0, double df = INFINITY);

protected:
  ReusableResource(std::string id, std::string name, T capacity,
                   T initial_level, ros::Duration latence = ros::Duration(0.0));
  ReusableResource(const ReusableResource<T>& resource);
};

template <typename T>
ReusableResource<T>::ReusableResource(std::string id, std::string name,
                                      T capacity, T initial_level,
                                      ros::Duration latence)
    : Resource<T>::Resource(id, false, name, capacity, initial_level, latence)
{
}

template <typename T>
ReusableResource<T>::ReusableResource(const ReusableResource<T>& resource)
    : Resource<T>::Resource(resource)
{
}

template <typename T> ReusableResource<T>::~ReusableResource() {}

template <typename T> void ReusableResource<T>::require(Task* task, T quantity, double d0, double df)
{
  if (task->hasStarted())
  {
    throw utilities::Exception("Unable to require the " + Resource<T>::str() +
                               " resource. The " + task->str() +
                               " task has already been started.");
  }
  utilities::functions::Function<T>* quantity_function;
  if (df < INFINITY)
  {
    quantity_function = new utilities::functions::PulseFunction<T>(d0, df, quantity, true, true);
  }
  else
  {
    quantity_function = new utilities::functions::StepFunction<T>(d0, quantity, true, true);
  }
  Resource<T>::profile_->addTaskFunction(
      new TaskFunction<T>(task, quantity_function));
}
}

#endif // _RULER_REUSABLE_RESOURCE_H_
