/**
 *  This header file implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/resource_reservation_request.h"
#include "ruler/task.h"

namespace ruler
{
Task::Task(std::string id, std::string name, ros::Duration expected_duration,
           bool preemptive)
    : utilities::Subject<TaskEvent>::Subject(id), name_(name),
      expected_duration_(expected_duration), preemptive_(preemptive),
      start_timestamp_bounds_(NULL), end_timestamp_bounds_(NULL)
{
}

Task::Task(const Task& task)
    : utilities::Subject<TaskEvent>::Subject(task), name_(task.name_),
      expected_duration_(task.expected_duration_),
      preemptive_(task.preemptive_), start_timestamp_(task.start_timestamp_),
      end_timestamp_(task.end_timestamp_),
      start_timestamp_bounds_(task.start_timestamp_bounds_),
      end_timestamp_bounds_(task.end_timestamp_bounds_),
      interruption_intervals_(task.interruption_intervals_)
{
}

Task::~Task()
{
  if (start_timestamp_bounds_)
  {
    delete start_timestamp_bounds_;
    start_timestamp_bounds_ = NULL;
  }
  if (end_timestamp_bounds_)
  {
    delete end_timestamp_bounds_;
    end_timestamp_bounds_ = NULL;
  }
  std::list<utilities::Interval<ros::Time>*>::iterator int_it(
      interruption_intervals_.begin());
  while (int_it != interruption_intervals_.end())
  {
    if (*int_it)
    {
      delete *int_it;
      *int_it = NULL;
    }
    int_it++;
  }
  std::list<ResourceReservationRequest*>::iterator req_it(
      resource_reservation_requests_.begin());
  while (req_it != resource_reservation_requests_.end())
  {
    if (*req_it)
    {
      delete *req_it;
      *req_it = NULL;
    }
    req_it++;
  }
}

void Task::addResourceReservationRequest(ResourceReservationRequest* request)
{
  resource_reservation_requests_.push_back(request);
}

void Task::start()
{
  std::list<ResourceReservationRequest*>::iterator it(
      resource_reservation_requests_.begin());
  while (it != resource_reservation_requests_.end())
  {
    ResourceReservationRequest* resource_reservation_request = *it;
    resource_reservation_request->request();
    it++;
  }
  if (hasStarted())
  {
    throw utilities::Exception(str() + " has already been started.");
  }
  if (utilities::Subject<TaskEvent>::empty())
  {
    throw utilities::Exception(str() +
                               " does not have any resource registered yet.");
  }
  start_timestamp_ = ros::Time::now();
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::STARTED));
  ROS_DEBUG_STREAM(*this << " has just started.");
}

void Task::interrupt()
{
  if (!hasStarted())
  {
    throw utilities::Exception(str() + " has not been started yet.");
  }
  if (hasFinished())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  if (isInterrupted())
  {
    throw utilities::Exception(str() + " has already been interrupted.");
  }
  last_interruption_timestamp_ = ros::Time::now();
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::INTERRUPTED));
  ROS_DEBUG_STREAM(*this << " has just interruped.");
}

void Task::resume()
{
  if (!hasStarted())
  {
    throw utilities::Exception(str() + " has not started yet.");
  }
  if (hasFinished())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  if (isRunning())
  {
    throw utilities::Exception(str() + " is already resuming.");
  }
  interruption_intervals_.push_back(new utilities::Interval<ros::Time>(
      last_interruption_timestamp_, ros::Time::now()));
  last_interruption_timestamp_ = ros::Time();
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::RESUMED));
  ROS_DEBUG_STREAM(*this << " has just resumed.");
}

void Task::finish()
{
  if (!hasStarted())
  {
    throw utilities::Exception(str() + " has not started yet.");
  }
  if (hasFinished())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  end_timestamp_ = ros::Time::now();
  if (!last_interruption_timestamp_.isZero())
  {
    interruption_intervals_.push_back(new utilities::Interval<ros::Time>(
        last_interruption_timestamp_, end_timestamp_));
  }
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::FINISHED));
  utilities::Subject<TaskEvent>::clearObservers();
  ROS_DEBUG_STREAM(*this << " has just finished.");
}

void Task::clearResources() { utilities::Subject<TaskEvent>::clearObservers(); }

double Task::getDuration(ros::Time t) const
{
  double duration(
      ((!end_timestamp_.isZero() && t > end_timestamp_ ? end_timestamp_ : t) -
       start_timestamp_).toSec());
  std::list<utilities::Interval<ros::Time>*>::const_iterator it(
      interruption_intervals_.begin());
  while (it != interruption_intervals_.end())
  {
    utilities::Interval<ros::Time>* interruption_interval = *it;
    if (t > interruption_interval->getMax())
    {
      duration -= (interruption_interval->getMax() -
                   interruption_interval->getMin()).toSec();
    }
    else if (interruption_interval->belongs(t))
    {
      duration -= (t - interruption_interval->getMin()).toSec();
    }
    it++;
  }
  if (!hasFinished() && isInterrupted())
  {
    duration -= (t - last_interruption_timestamp_).toSec();
  }
  return duration;
}

std::string Task::getName() const { return name_; }

ros::Duration Task::getExpectedDuration() const { return expected_duration_; }

bool Task::isPreemptive() const { return preemptive_; }

ros::Time Task::getStartTimestamp() const { return start_timestamp_; }

ros::Time Task::getEndTimestamp() const { return end_timestamp_; }

bool Task::hasStarted() const { return !start_timestamp_.isZero(); }

bool Task::isInterrupted() const
{
  return hasStarted() && !last_interruption_timestamp_.isZero();
}

bool Task::isRunning() const
{
  return hasStarted() && last_interruption_timestamp_.isZero();
}

bool Task::hasFinished() const { return !end_timestamp_.isZero(); }
}
