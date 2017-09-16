/**
 *  This header file defines the ResourceReservationRequest abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_RESERVATION_REQUEST_H_
#define _RULER_RESOURCE_RESERVATION_REQUEST_H_

#include <boost/shared_ptr.hpp>

namespace ruler
{
class Task;
typedef boost::shared_ptr<Task> TaskPtr;
typedef boost::shared_ptr<Task const> TaskConstPtr;

class ResourceReservationRequest
{
public:
  ResourceReservationRequest(const TaskPtr& task);
  ResourceReservationRequest(const ResourceReservationRequest& request);
  virtual ~ResourceReservationRequest();
  TaskPtr getTask() const;
  virtual void request() = 0;

protected:
  TaskPtr task_;
};

typedef boost::shared_ptr<ResourceReservationRequest>
    ResourceReservationRequestPtr;
typedef boost::shared_ptr<ResourceReservationRequest const>
    ResourceReservationRequestConstPtr;
}

#endif // _RULER_RESOURCE_RESERVATION_REQUEST_H_
