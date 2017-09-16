#include "ruler/preemptive_task.h"

namespace ruler
{

void Task::interrupt(const ros::Time &timestamp)
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

void Task::resume(const ros::Time &timestamp)
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
  interruption_intervals_.push_back(new utilities::Interval<ros::Time>(
      last_interruption_timestamp_, timestamp));
  last_interruption_timestamp_ = ros::Time();
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::RESUMED, timestamp));
  utilities::Subject::notify(event);
  ROS_DEBUG_STREAM(*this << " has just resumed.");
  last_event_timestamp_ = timestamp;
}

double Task::getDuration(const ros::Time &timestamp) const
{
  double duration(
      ((!end_timestamp_.isZero() && timestamp > end_timestamp_ ? end_timestamp_ : timestamp) -
       start_timestamp_).toSec());
  std::list<utilities::Interval<ros::Time>*>::const_iterator it(
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
  }
  if (!hasFinished() && isInterrupted())
  {
    duration -= (timestamp - last_interruption_timestamp_).toSec();
  }
  return duration;
}

ros::Time Task::getLastInterruptionTimestamp() const
{
  if (isInterrupted())
  {
    return last_interruption_timestamp_;
  }
  utilities::Interval<ros::Time>* last_interval(interruption_intervals_.back());
  return !interruption_intervals_.empty() ? last_interval->getMin()
                                          : ros::Time();
}

ros::Time Task::getLastResumeTimestamp() const
{
  utilities::Interval<ros::Time>* last_interval(interruption_intervals_.back());
  return !isInterrupted() && !interruption_intervals_.empty()
             ? last_interval->getMax()
             : ros::Time();
}

bool Task::isInterrupted() const
{
  return hasStarted() && !last_interruption_timestamp_.isZero();
}

bool Task::isRunning() const
{
  return hasStarted() && last_interruption_timestamp_.isZero();
}
}
