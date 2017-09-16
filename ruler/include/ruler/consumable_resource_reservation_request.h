/**
 *  This header file defines and implements the
 *ConsumableResourceReservationRequest class, which is based on the
 *ResourceReservationRequest abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONSUMABLE_RESOURCE_RESERVATION_REQUEST_H_
#define _RULER_CONSUMABLE_RESOURCE_RESERVATION_REQUEST_H_

#include <ros/common.h>
#include "ruler/unary_consumable_resource.h"
#include "ruler/resource_reservation_request.h"
#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/functions/unary_step_function.h"

namespace ruler
{
template <typename T>
class ConsumableResourceReservationRequest : public ResourceReservationRequest
{
protected:
  typedef typename ConsumableResource<T>::Ptr ConsumableResourcePtr;
  typedef typename ConsumableResource<T>::ConstPtr ConsumableResourceConstPtr;
  typedef typename utilities::functions::Function<T>::Ptr FunctionPtr;
  typedef typename utilities::functions::Function<T>::ConstPtr FunctionConstPtr;

public:
  typedef boost::shared_ptr<ConsumableResourceReservationRequest<T> > Ptr;
  typedef boost::shared_ptr<ConsumableResourceReservationRequest<T> const> ConstPtr;
  ConsumableResourceReservationRequest(
      const TaskPtr& task, const UnaryConsumableResourcePtr& resource,
      double d0 = 0.0, bool consumption = true);
  ConsumableResourceReservationRequest(const TaskPtr& task,
                                       const ConsumableResourcePtr& resource,
                                       const FunctionPtr& quantity_function,
                                       bool consumption = true);
  ConsumableResourceReservationRequest(
      const ConsumableResourceReservationRequest<T>& request);
  virtual ~ConsumableResourceReservationRequest();
  virtual ConsumableResourcePtr getResource() const;
  virtual bool isConsumption() const;
  virtual bool isProduction() const;
  virtual void request();

private:
  ConsumableResourcePtr resource_;
  FunctionPtr quantity_function_;
  const bool consumption_;
};

typedef ConsumableResourceReservationRequest<utilities::ContinuousSignalType>
    ContinuousConsumableResourceReservationRequest;
typedef boost::shared_ptr<ContinuousConsumableResourceReservationRequest>
    ContinuousConsumableResourceReservationRequestPtr;
typedef boost::shared_ptr<ContinuousConsumableResourceReservationRequest const>
    ContinuousConsumableResourceReservationRequestConstPtr;
typedef ConsumableResourceReservationRequest<utilities::DiscreteSignalType>
    DiscreteConsumableResourceReservationRequest;
typedef boost::shared_ptr<DiscreteConsumableResourceReservationRequest>
    DiscreteConsumableResourceReservationRequestPtr;
typedef boost::shared_ptr<DiscreteConsumableResourceReservationRequest const>
    DiscreteConsumableResourceReservationRequestConstPtr;
typedef ConsumableResourceReservationRequest<utilities::UnarySignalType>
    UnaryConsumableResourceReservationRequest;
typedef boost::shared_ptr<UnaryConsumableResourceReservationRequest>
    UnaryConsumableResourceReservationRequestPtr;
typedef boost::shared_ptr<UnaryConsumableResourceReservationRequest const>
    UnaryConsumableResourceReservationRequestConstPtr;

template <typename T>
ConsumableResourceReservationRequest<T>::ConsumableResourceReservationRequest(
    const TaskPtr& task, const UnaryConsumableResourcePtr& resource, double d0,
    bool consumption)
    : ResourceReservationRequest::ResourceReservationRequest(task),
      resource_(resource),
      quantity_function_(new utilities::functions::UnaryStepFunction(d0)),
      consumption_(consumption)
{
}

template <typename T>
ConsumableResourceReservationRequest<T>::ConsumableResourceReservationRequest(
    const TaskPtr& task, const ConsumableResourcePtr& resource,
    const FunctionPtr& quantity_function, bool consumption)
    : ResourceReservationRequest::ResourceReservationRequest(task),
      resource_(resource), quantity_function_(quantity_function),
      consumption_(consumption)
{
  if (resource->isUnary())
  {
    throw utilities::Exception("Use the other "
                               "ConsumableResourceReservationRequest "
                               "constructor for UnarySignalType.");
  }
}

template <typename T>
ConsumableResourceReservationRequest<T>::ConsumableResourceReservationRequest(
    const ConsumableResourceReservationRequest<T>& request)
    : ResourceReservationRequest::ResourceReservationRequest(request),
      resource_(request.resource_),
      quantity_function_(request.quantity_function_),
      consumption_(request.consumption_)
{
}

template <typename T>
ConsumableResourceReservationRequest<T>::~ConsumableResourceReservationRequest()
{
}

template <typename T>
typename ConsumableResource<T>::Ptr
ConsumableResourceReservationRequest<T>::getResource() const
{
  return resource_;
}

template <typename T>
bool ConsumableResourceReservationRequest<T>::isConsumption() const
{
  return consumption_;
}

template <typename T>
bool ConsumableResourceReservationRequest<T>::isProduction() const
{
  return !consumption_;
}

template <typename T> void ConsumableResourceReservationRequest<T>::request()
{
  task_->addResource(resource_);
  if (consumption_)
  {
    resource_->consume(task_, quantity_function_);
    ROS_DEBUG_STREAM("Requested " << resource_
                                  << " resource consumption during " << task_
                                  << " task execution.");
  }
  else
  {
    resource_->produce(task_, quantity_function_);
    ROS_DEBUG_STREAM("Requested " << resource_ << " resource production during "
                                  << task_ << " task execution.");
  }
}
}

#endif // _RULER_CONSUMABLE_RESOURCE_RESERVATION_REQUEST_H_
