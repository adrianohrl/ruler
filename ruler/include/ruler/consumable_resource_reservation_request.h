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
#include "ruler/consumable_resource.h"
#include "ruler/resource_reservation_request.h"
#include "utilities/functions/unary_step_function.h"

namespace ruler
{
template <typename T>
class ConsumableResourceReservationRequest : public ResourceReservationRequest
{
public:
  ConsumableResourceReservationRequest(
      Task* task, ConsumableResource<utilities::UnarySignalType>* resource,
      double d0 = 0.0, bool consumption = true);
  ConsumableResourceReservationRequest(
      Task* task, ConsumableResource<T>* resource,
      utilities::functions::Function<T>* quantity_function,
      bool consumption = true);
  ConsumableResourceReservationRequest(
      const ConsumableResourceReservationRequest<T>& request);
  virtual ~ConsumableResourceReservationRequest();
  virtual ConsumableResource<T>* getResource() const;
  virtual bool isConsumption() const;
  virtual bool isProduction() const;
  virtual void request();

private:
  ConsumableResource<T>* resource_;
  utilities::functions::Function<T>* quantity_function_;
  const bool consumption_;
};

template <typename T>
ConsumableResourceReservationRequest<T>::ConsumableResourceReservationRequest(
    Task* task, ConsumableResource<utilities::UnarySignalType>* resource,
    double d0, bool consumption)
    : ResourceReservationRequest::ResourceReservationRequest(task),
      resource_(resource),
      quantity_function_(new utilities::functions::UnaryStepFunction(d0)),
      consumption_(consumption)
{
}

template <typename T>
ConsumableResourceReservationRequest<T>::ConsumableResourceReservationRequest(
    Task* task, ConsumableResource<T>* resource,
    utilities::functions::Function<T>* quantity_function, bool consumption)
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
  resource_ = NULL;
  quantity_function_ = NULL;
}

template <typename T>
ConsumableResource<T>*
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
