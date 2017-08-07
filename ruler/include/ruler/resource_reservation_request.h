/**
 *  This header file defines the ResourceReservationRequest abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_RESERVATION_REQUEST_H_
#define _RULER_RESOURCE_RESERVATION_REQUEST_H_

namespace ruler
{
class Task;

class ResourceReservationRequest
{
public:
  ResourceReservationRequest(Task* task);
  ResourceReservationRequest(const ResourceReservationRequest& request);
  virtual ~ResourceReservationRequest();
  Task* getTask() const;
  virtual void request() = 0;

protected:
  Task* task_;
};
}

#endif // _RULER_RESOURCE_RESERVATION_REQUEST_H_
