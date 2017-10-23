/**
 *  This header file defines and implements the ConsumableResource abstract
 *class, which is based on the Resource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONSUMABLE_RESOURCE_H_
#define _RULER_CONSUMABLE_RESOURCE_H_

#include <boost/enable_shared_from_this.hpp>
#include "utilities/functions/function.h"
#include "ruler/resource.h"

namespace ruler
{
template <typename T> class ConsumableResource : public Resource<T>
{
protected:
  typedef typename boost::enable_shared_from_this<Resource<T> >
      enable_shared_from_this;
  typedef typename utilities::functions::Function<T>::Ptr FunctionPtr;
  typedef typename utilities::functions::Function<T>::ConstPtr FunctionConstPtr;
  typedef typename TaskFunction<T>::Ptr TaskFunctionPtr;
  typedef typename TaskFunction<T>::ConstPtr TaskFunctionConstPtr;

public:
  typedef boost::shared_ptr<ConsumableResource<T> > Ptr;
  typedef boost::shared_ptr<ConsumableResource<T> const> ConstPtr;
  ConsumableResource(const std::string& id, const std::string& name,
                     const T& capacity, const T& initial_level,
                     const ros::Duration& latence = ros::Duration(0.0));
  ConsumableResource(const ruler_msgs::Resource& msg);
  ConsumableResource(const ConsumableResource<T>& resource);
  virtual ~ConsumableResource();
  virtual bool isConsumable() const;
  virtual void consume(const TaskPtr& task,
                       const FunctionPtr& quantity_function);
  virtual void consume(const TaskPtr& task, const T& quantity, double d0 = 0.0,
                       double df = INFINITY);
  virtual void produce(const TaskPtr& task,
                       const FunctionPtr& quantity_function);
  virtual void produce(const TaskPtr& task, const T& quantity, double d0 = 0.0,
                       double df = INFINITY);
};

typedef ConsumableResource<utilities::ContinuousSignalType>
    ContinuousConsumableResource;
typedef boost::shared_ptr<ContinuousConsumableResource>
    ContinuousConsumableResourcePtr;
typedef boost::shared_ptr<ContinuousConsumableResource const>
    ContinuousConsumableResourceConstPtr;
typedef ConsumableResource<utilities::DiscreteSignalType>
    DiscreteConsumableResource;
typedef boost::shared_ptr<DiscreteConsumableResource>
    DiscreteConsumableResourcePtr;
typedef boost::shared_ptr<DiscreteConsumableResource const>
    DiscreteConsumableResourceConstPtr;

template <typename T>
ConsumableResource<T>::ConsumableResource(const std::string& id,
                                          const std::string& name,
                                          const T& capacity,
                                          const T& initial_level,
                                          const ros::Duration& latence)
    : Resource<T>::Resource(id, name, capacity, initial_level, latence)
{
}

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
void ConsumableResource<T>::consume(const TaskPtr& task,
                                    const FunctionPtr& quantity_function)
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
  TaskFunctionPtr task_function(new TaskFunction<T>(
      enable_shared_from_this::shared_from_this(), task, quantity_function));
  Resource<T>::profile_->addTaskFunction(task_function);
}

template <typename T>
void ConsumableResource<T>::consume(const TaskPtr& task, const T& quantity,
                                    double d0, double df)
{
  FunctionPtr quantity_function;
  if (df == INFINITY)
  {
    quantity_function.reset(
        new utilities::functions::StepFunction<T>(d0, (double) quantity));
  }
  else
  {
    quantity_function.reset(
        new utilities::functions::PulseFunction<T>(d0, df, (double) quantity));
  }
  consume(task, quantity_function);
}

template <typename T>
void ConsumableResource<T>::produce(const TaskPtr& task,
                                    const FunctionPtr& quantity_function)
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
  TaskFunctionPtr task_function(new TaskFunction<T>(
      enable_shared_from_this::shared_from_this(), task, quantity_function));
  Resource<T>::profile_->addTaskFunction(task_function);
}

template <typename T>
void ConsumableResource<T>::produce(const TaskPtr& task, const T& quantity,
                                    double d0, double df)
{
  FunctionPtr quantity_function;
  if (df == INFINITY)
  {
    quantity_function.reset(
        new utilities::functions::StepFunction<T>(d0, (double) quantity));
  }
  else
  {
    quantity_function.reset(
        new utilities::functions::PulseFunction<T>(d0, df, (double) quantity));
  }
  produce(task, quantity_function);
}
}

#endif // _RULER_CONSUMABLE_RESOURCE_H_
