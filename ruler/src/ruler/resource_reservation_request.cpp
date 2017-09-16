/**
 *  This source file implements the ResourceReservationRequest abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/resource_reservation_request.h"
#include "ruler/task.h"

namespace ruler
{
ResourceReservationRequest::ResourceReservationRequest(const TaskPtr& task)
    : task_(task)
{
}

ResourceReservationRequest::ResourceReservationRequest(
    const ResourceReservationRequest& request)
    : task_(request.task_)
{
}

ResourceReservationRequest::~ResourceReservationRequest() {}

TaskPtr ResourceReservationRequest::getTask() const { return task_; }
}
