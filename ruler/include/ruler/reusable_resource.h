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
  typedef boost::shared_ptr<ReusableResource<T> > Ptr;
  typedef boost::shared_ptr<ReusableResource<T> const> ConstPtr;
  ReusableResource(const ruler_msgs::Resource& msg);
  ReusableResource(const std::string& id, const std::string& name,
                   const T& capacity, const T& initial_level,
                   const ros::Duration& latence = ros::Duration(0.0));
  ReusableResource(const ReusableResource<T>& resource);
  virtual ~ReusableResource();
  virtual bool isReusable() const;
  virtual void require(const TaskPtr& task, const T& quantity, double d0 = 0.0,
                       double df = INFINITY);

protected:
  typedef typename boost::enable_shared_from_this<Resource<T> > enable_shared_from_this;
  typedef typename TaskFunction<T>::Ptr TaskFunctionPtr;
  typedef typename TaskFunction<T>::ConstPtr TaskFunctionConstPtr;
};

/*typedef ReusableResource<utilities::ContinuousSignalType>
    ContinuousReusableResource;
typedef boost::shared_ptr<ContinuousReusableResource>
    ContinuousReusableResourcePtr;
typedef boost::shared_ptr<ContinuousReusableResource const>
    ContinuousReusableResourceConstPtr;
typedef ReusableResource<utilities::DiscreteSignalType>
    DiscreteReusableResource;
typedef boost::shared_ptr<DiscreteReusableResource>
    DiscreteReusableResourcePtr;
typedef boost::shared_ptr<DiscreteReusableResource const>
    DiscreteReusableResourceConstPtr;*//*
typedef ReusableResource<utilities::UnarySignalType>
    UnaryReusableResource;
typedef boost::shared_ptr<UnaryReusableResource>
    UnaryReusableResourcePtr;
typedef boost::shared_ptr<UnaryReusableResource const>
    UnaryReusableResourceConstPtr;*/

template <typename T>
ReusableResource<T>::ReusableResource(const ruler_msgs::Resource& msg)
    : Resource<T>::Resource(msg)
{
  if (msg.consumable)
  {
    throw utilities::Exception("Not a reusable resource ros message.");
  }
}

template <typename T>
ReusableResource<T>::ReusableResource(const std::string& id,
                                      const std::string& name,
                                      const T& capacity, const T& initial_level,
                                      const ros::Duration& latence)
    : Resource<T>::Resource(id, name, capacity, initial_level, latence)
{
}

template <typename T>
ReusableResource<T>::ReusableResource(const ReusableResource<T>& resource)
    : Resource<T>::Resource(resource)
{
}

template <typename T> ReusableResource<T>::~ReusableResource() {}

template <typename T> bool ReusableResource<T>::isReusable() const
{
  return true;
}

template <typename T>
void ReusableResource<T>::require(const TaskPtr& task, const T& quantity,
                                  double d0, double df)
{
  typedef typename utilities::functions::Function<T>::Ptr FunctionPtr;
  typedef utilities::functions::PulseFunction<T> PulseFunction;
  typedef utilities::functions::StepFunction<T> StepFunction;
  if (task->hasStarted())
  {
    throw utilities::Exception("Unable to require the " + Resource<T>::str() +
                               " resource. The " + task->str() +
                               " task has already been started.");
  }
  FunctionPtr quantity_function;
  if (df < INFINITY)
  {
    quantity_function.reset(new PulseFunction(d0, df, quantity, true, true));
  }
  else
  {
    quantity_function.reset(new StepFunction(d0, quantity, true, true));
  }
  TaskFunctionPtr task_function(
        new TaskFunction<T>(enable_shared_from_this::shared_from_this(),
                            task, quantity_function));
  Resource<T>::profile_->addTaskFunction(task_function);
}
}

#endif // _RULER_REUSABLE_RESOURCE_H_
