/**
 *  This header file implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/resource_reservation_request.h"
#include "ruler/task.h"
#include "utilities/exception.h"

namespace ruler
{
Task::Task(const std::string& id, const std::string& name,
           const utilities::NoisyTimePtr& expected_start,
           const utilities::NoisyTimePtr& expected_end, bool preemptive,
           const std::list<geometry_msgs::Pose>& waypoints)
    : Subject::Subject(id), name_(name), preemptive_(preemptive), expected_start_(expected_start),
      expected_end_(expected_end), waypoints_(waypoints),
      expected_duration_(
          new utilities::NoisyDuration(expected_end_->getMean() - expected_start_->getMean(),
                                       (expected_start_->getStandardDeviation() +
                                        expected_end_->getStandardDeviation()).toSec()))
{
}

Task::Task(const ruler_msgs::Task& msg)
    : Subject::Subject(msg.header.frame_id), name_(msg.name),
      preemptive_(msg.preemptive)
    // arrumar aki
{
  /*if (msg.min_interruption_timestamps.size() !=
      msg.max_interruption_timestamps.size())
  {
    throw utilities::Exception("The number of the minimum and maximum "
                               "interruptions must be the same.");
  }
  start_timestamp_bounds_ = new utilities::Interval<ros::Time>(
      msg.min_start_timestamp, msg.max_start_timestamp);
  end_timestamp_bounds_ = new utilities::Interval<ros::Time>(
      msg.min_end_timestamp, msg.max_end_timestamp);
  for (int i(0); i < msg.min_interruption_timestamps.size(); i++)
  {
    interruption_intervals_.push_back(
        new utilities::Interval<ros::Time>(msg.min_interruption_timestamps[i],
                                           msg.max_interruption_timestamps[i]));
  }
  for (int i(0); i < msg.waypoints.size(); i++)
  {
    waypoints_.push_back(msg.waypoints[i]);
  }*/
}

Task::Task(const Task& task)
    : Subject::Subject(task), name_(task.name_),
      expected_duration_(task.expected_duration_),
      preemptive_(task.preemptive_), start_timestamp_(task.start_timestamp_),
      end_timestamp_(task.end_timestamp_),
      expected_start_(task.expected_start_),
      expected_end_(task.expected_end_),
      interruption_intervals_(task.interruption_intervals_),
      waypoints_(task.waypoints_)
{
}

Task::~Task() {}

void Task::addResourceReservationRequest(const ResourceReservationRequestPtr& request)
{
  if (*this != *request->getTask())
  {
    throw utilities::Exception("The input request does not belong to " + str() +
                               ".");
  }
  resource_reservation_requests_.push_back(request);
}

void Task::addResource(const ResourceInterfacePtr& resource)
{
  if (hasStarted())
  {
    throw utilities::Exception("Unable to add the " + resource->str() +
                               " resource. The " + str() +
                               " is already running.");
  }
  utilities::Subject::registerObserver(resource);
}

void Task::removeResource(const ResourceInterfacePtr& resource)
{
  if (hasStarted() && !hasFinished())
  {
    throw utilities::Exception("Unable to remove the " + resource->str() +
                               " resource. The " + str() +
                               " is still running.");
  }
  utilities::Subject::unregisterObserver(resource);
}

void Task::start(const ros::Time& timestamp)
{
  if (timestamp <= last_event_timestamp_)
  {
    throw utilities::Exception(
        "The input timestamp must be after the last event timestamp.");
  }
  /*std::list<ResourceReservationRequest*>::iterator it(
      resource_reservation_requests_.begin());
  while (it != resource_reservation_requests_.end())
  {
    ResourceReservationRequest* resource_reservation_request = *it;
    resource_reservation_request->request();
    it++;
  }*/
  if (hasStarted())
  {
    throw utilities::Exception(str() + " has already been started.");
  }
  if (utilities::Subject::empty())
  {
    throw utilities::Exception(str() +
                               " does not have any resource registered yet.");
  }
  start_timestamp_ = timestamp;
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::STARTED, timestamp));
  utilities::Subject::notify(event);
  ROS_DEBUG_STREAM(*this << " has just started.");
  last_event_timestamp_ = timestamp;
}

void Task::interrupt(const ros::Time& timestamp)
{
  if (!preemptive_)
  {
    throw utilities::Exception(
        "This task cannot be interrupted because it is not preemptive.");
  }
  if (timestamp <= last_event_timestamp_)
  {
    throw utilities::Exception(
        "The input timestamp must be after the last event timestamp.");
  }
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
  last_interruption_timestamp_ = timestamp;
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::INTERRUPTED, timestamp));
  utilities::Subject::notify(event);
  ROS_DEBUG_STREAM(*this << " has just interruped.");
  last_event_timestamp_ = timestamp;
}

void Task::resume(const ros::Time& timestamp)
{
  if (!preemptive_)
  {
    throw utilities::Exception(
        "This task cannot be resumed because it is not preemptive.");
  }
  if (timestamp <= last_event_timestamp_)
  {
    throw utilities::Exception(
        "The input timestamp must be after the last event timestamp.");
  }
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
  /*interruption_intervals_.push_back(new utilities::Interval<ros::Time>(
      last_interruption_timestamp_, timestamp));*/
  last_interruption_timestamp_ = ros::Time();
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::RESUMED, timestamp));
  utilities::Subject::notify(event);
  ROS_DEBUG_STREAM(*this << " has just resumed.");
  last_event_timestamp_ = timestamp;
}

void Task::finish(const ros::Time& timestamp)
{
  if (timestamp <= last_event_timestamp_)
  {
    throw utilities::Exception(
        "The input timestamp must be after the last event timestamp.");
  }
  if (!hasStarted())
  {
    throw utilities::Exception(str() + " has not started yet.");
  }
  if (hasFinished())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  end_timestamp_ = timestamp;
  if (!last_interruption_timestamp_.isZero())
  {
    /*interruption_intervals_.push_back(new utilities::Interval<ros::Time>(
        last_interruption_timestamp_, end_timestamp_));*/
  }
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::FINISHED, timestamp));
  utilities::Subject::notify(event);
  utilities::Subject::clearObservers();
  ROS_DEBUG_STREAM(*this << " has just finished.");
  last_event_timestamp_ = timestamp;
}

void Task::clearResources() { utilities::Subject::clearObservers(); }

double Task::getDuration(const ros::Time& timestamp) const
{
  double duration(
      ((!end_timestamp_.isZero() && timestamp > end_timestamp_ ? end_timestamp_
                                                               : timestamp) -
       start_timestamp_).toSec());
  /*std::list<utilities::Interval<ros::Time>*>::const_iterator it(
      interruption_intervals_.begin());
  while (it != interruption_intervals_.end())
  {
    utilities::Interval<ros::Time>* interruption_interval = *it;
    if (timestamp > interruption_interval->getMax())
    {
      duration -= (interruption_interval->getMax() -
                   interruption_interval->getMin()).toSec();
    }
    else if (interruption_interval->belongs(timestamp))
    {
      duration -= (timestamp - interruption_interval->getMin()).toSec();
    }
    it++;
  }*/
  if (!hasFinished() && isInterrupted())
  {
    duration -= (timestamp - last_interruption_timestamp_).toSec();
  }
  return duration;
}

std::string Task::getName() const { return name_; }

utilities::NoisyDurationPtr Task::getExpectedDuration() const { return expected_duration_; }

bool Task::isPreemptive() const { return preemptive_; }

ros::Time Task::getStartTimestamp() const { return start_timestamp_; }

ros::Time Task::getLastInterruptionTimestamp() const
{
  if (isInterrupted())
  {
    return last_interruption_timestamp_;
  }
  /*utilities::Interval<ros::Time>* last_interval(interruption_intervals_.back());
  return !interruption_intervals_.empty() ? last_interval->getMin()
                                          : ros::Time();*/
}

ros::Time Task::getLastResumeTimestamp() const
{
  /*utilities::Interval<ros::Time>* last_interval(interruption_intervals_.back());
  return !isInterrupted() && !interruption_intervals_.empty()
             ? last_interval->getMax()
             : ros::Time();*/
}

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

utilities::NoisyTimePtr Task::getExpectedStart() const
{
  return expected_start_;
}

utilities::NoisyTimePtr Task::getExpectedEnd() const
{
  return expected_end_;
  ;
}

std::list<geometry_msgs::Pose> Task::getWaypoints() const { return waypoints_; }

double Task::getDistance() const
{
  if (waypoints_.size() <= 1)
  {
    return 0.0;
  }
  std::list<geometry_msgs::Pose>::const_iterator it(waypoints_.begin());
  geometry_msgs::Pose p1, p2;
  p1 = *it;
  it++;
  double distance(0.0);
  while (it != waypoints_.end())
  {
    p2 = *it;
    distance += sqrt(pow(p2.position.x - p1.position.x, 2) +
                     pow(p2.position.y - p1.position.y, 2));
    it++;
    p1 = p2;
  }
  return distance;
}

ruler_msgs::Task Task::toMsg() const
{
  ruler_msgs::Task msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = getId();
  msg.name = name_;
  //msg.expected_duration = expected_duration_;
  msg.start_timestamp = start_timestamp_;
  msg.end_timestamp = end_timestamp_;
  /*msg.min_start_timestamp = start_timestamp_bounds_->getMin();
  msg.max_start_timestamp = start_timestamp_bounds_->getMax();
  msg.min_end_timestamp = end_timestamp_bounds_->getMin();
  msg.max_end_timestamp = end_timestamp_bounds_->getMax();
  std::list<utilities::Interval<ros::Time>*>::const_iterator intervals_it(
      interruption_intervals_.begin());
  while (intervals_it != interruption_intervals_.end())
  {
    utilities::Interval<ros::Time>* interruption_interval = *intervals_it;
    msg.min_interruption_timestamps.push_back(interruption_interval->getMin());
    msg.max_interruption_timestamps.push_back(interruption_interval->getMax());
    intervals_it++;
  }*/
  std::list<geometry_msgs::Pose>::const_iterator waypoints_it(
      waypoints_.begin());
  while (waypoints_it != waypoints_.end())
  {
    msg.waypoints.push_back(*waypoints_it);
    waypoints_it++;
  }
  return msg;
}

bool Task::operator==(const ruler_msgs::Task& msg) const
{
  return getId() == msg.header.frame_id;
}
}
