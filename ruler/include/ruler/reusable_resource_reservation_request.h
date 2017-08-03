/**
 *  This header file defines and implements the
 *ReusableResourceReservationRequest
 *class, which is based on the ResourceReservationRequest abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_REUSABLE_RESOURCE_RESERVATION_REQUEST_H_
#define _RULER_REUSABLE_RESOURCE_RESERVATION_REQUEST_H_

#include "ruler/resource_reservation_request.h"
#include "ruler/reusable_resource.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
template <typename T>
class ReusableResourceReservationRequest : public ResourceReservationRequest
{
public:
  ReusableResourceReservationRequest(
      Task* task, ReusableResource<utilities::UnarySignalType>* resource,
      double d0 = 0.0, double df = INFINITY);
  ReusableResourceReservationRequest(Task* task, ReusableResource<T>* resource,
                                     T quantity, double d0 = 0.0,
                                     double df = INFINITY);
  ReusableResourceReservationRequest(
      const ReusableResourceReservationRequest<T>& request);
  virtual ~ReusableResourceReservationRequest();
  virtual ReusableResource<T>* getResource() const;
  virtual void request();

private:
  ReusableResource<T>* resource_;
  T quantity_;
  double d0_;
  double df_;
};

template <typename T>
ReusableResourceReservationRequest<T>::ReusableResourceReservationRequest(
    Task* task, ReusableResource<utilities::UnarySignalType>* resource,
    double d0, double df)
    : ResourceReservationRequest::ResourceReservationRequest(task),
      resource_(resource), quantity_(utilities::UnarySignalType(true)), d0_(d0),
      df_(df)
{
}

template <typename T>
ReusableResourceReservationRequest<T>::ReusableResourceReservationRequest(
    Task* task, ReusableResource<T>* resource, T quantity, double d0, double df)
    : ResourceReservationRequest::ResourceReservationRequest(task),
      resource_(resource), quantity_(quantity), d0_(d0), df_(df)
{
  if (resource->isUnary())
  {
    throw utilities::Exception("Use the other "
                               "ReusableResourceReservationRequest constructor "
                               "for UnarySignalType.");
  }
}

template <typename T>
ReusableResourceReservationRequest<T>::ReusableResourceReservationRequest(
    const ReusableResourceReservationRequest<T>& request)
    : ResourceReservationRequest::ResourceReservationRequest(request),
      resource_(request.resource_), quantity_(request.quantity_),
      d0_(request.d0_), df_(request.df_)
{
}

template <typename T>
ReusableResourceReservationRequest<T>::~ReusableResourceReservationRequest()
{
  resource_ = NULL;
}

template <typename T>
ReusableResource<T>* ReusableResourceReservationRequest<T>::getResource() const
{
  return resource_;
}

template <typename T> void ReusableResourceReservationRequest<T>::request()
{
  task_->addResource(resource_);
  resource_->require(task_, quantity_, d0_, df_);
}
}

#endif // _RULER_REUSABLE_RESOURCE_RESERVATION_REQUEST_H_
