/**
 *  This header file defines the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_H_
#define _RULER_TASK_H_

#include <ros/time.h>
#include "ruler/task_event.h"
#include "utilities/interval.h"
#include "utilities/subject.h"

namespace ruler
{
class ResourceReservationRequest;

template <typename T> class Resource;

class Task : public utilities::Subject<TaskEvent>
{
public:
  Task(std::string id, std::string name, ros::Duration expected_duration,
       bool preemptive = false);
  Task(const Task& task);
  virtual ~Task();
  void addResourceReservationRequest(ResourceReservationRequest* request);
  template <typename T> void addResource(Resource<T>* resource);
  template <typename T> void removeResource(const Resource<T>& resource);
  void start();
  void interrupt();
  void resume();
  void finish();
  void clearResources();
  double getDuration(ros::Time t = ros::Time::now()) const;
  std::string getName() const;
  ros::Duration getExpectedDuration() const;
  bool isPreemptive() const;
  ros::Time getStartTimestamp() const;
  ros::Time getEndTimestamp() const;
  bool hasStarted() const;
  bool isInterrupted() const;
  bool isRunning() const;
  bool hasFinished() const;

private:
  std::string name_;
  ros::Duration expected_duration_;
  bool preemptive_;
  ros::Time start_timestamp_;
  ros::Time last_interruption_timestamp_;
  ros::Time end_timestamp_;
  utilities::Interval<ros::Time>* start_timestamp_bounds_;
  utilities::Interval<ros::Time>* end_timestamp_bounds_;
  std::list<utilities::Interval<ros::Time>*> interruption_intervals_;
  std::list<ResourceReservationRequest*> resource_reservation_requests_;
};
}

#include "ruler/resource.h"

namespace ruler
{
template <typename T> void Task::addResource(Resource<T>* resource)
{
  if (hasStarted())
  {
    throw utilities::Exception("Unable to add the " + resource->str() +
                               " resource. The " + str() +
                               " is already running.");
  }
  utilities::Subject<TaskEvent>::registerObserver(resource);
}

template <typename T> void Task::removeResource(const Resource<T>& resource)
{
  if (hasStarted() && !hasFinished())
  {
    throw utilities::Exception("Unable to remove the " + resource->str() +
                               " resource. The " + str() +
                               " is still running.");
  }
  utilities::Subject<TaskEvent>::unregisterObserver(resource);
}
}

#endif // _RULER_TASK_H_
