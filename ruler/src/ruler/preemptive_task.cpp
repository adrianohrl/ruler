#include "ruler/preemptive_task.h"

namespace ruler
{

PreemptiveTask::PreemptiveTask(const std::string& id, const std::string& name,
                               const utilities::NoisyTimePtr& expected_start,
                               const utilities::NoisyTimePtr& expected_end,
                               const std::list<geometry_msgs::Pose>& waypoints)
    : Task::Task(id, name, expected_start, expected_end, waypoints)
{
}

PreemptiveTask::PreemptiveTask(const ruler_msgs::Task& msg) : Task::Task(msg)
{
  if (!msg.preemptive)
  {
    throw utilities::Exception("This is a non-preemptive task message.");
  }
}

PreemptiveTask::PreemptiveTask(const PreemptiveTask& task) : Task::Task(task) {}

PreemptiveTask::~PreemptiveTask() {}

void PreemptiveTask::interrupt(const ros::Time& timestamp)
{
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

void PreemptiveTask::resume(const ros::Time& timestamp)
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
  if (isRunning())
  {
    throw utilities::Exception(str() + " is already resuming.");
  }
  interruptions_.push_back(utilities::TimeIntervalPtr(
      new utilities::TimeInterval(last_interruption_timestamp_, timestamp)));
  last_interruption_timestamp_ = ros::Time();
  TaskEventConstPtr event(
      new TaskEvent(shared_from_this(), types::RESUMED, timestamp));
  utilities::Subject::notify(event);
  ROS_DEBUG_STREAM(*this << " has just resumed.");
  last_event_timestamp_ = timestamp;
}

void PreemptiveTask::finish(const ros::Time& timestamp)
{
  Task::finish(timestamp);
  if (!last_interruption_timestamp_.isZero())
  {
    interruptions_.push_back(
        utilities::TimeIntervalPtr(new utilities::TimeInterval(
            last_interruption_timestamp_, end_timestamp_)));
  }
}

ros::Duration PreemptiveTask::getDuration(const ros::Time& timestamp) const
{
  ros::Duration duration(Task::getDuration(timestamp));
  for (interruptions_const_iterator it(interruptions_.begin());
       it != interruptions_.end(); it++)
  {
    utilities::TimeIntervalPtr interruption(*it);
    if (timestamp > interruption->getMax())
    {
      duration -= interruption->getMax() - interruption->getMin();
    }
    else if (interruption->belongs(timestamp))
    {
      duration -= timestamp - interruption->getMin();
    }
  }
  if (!hasFinished() && isInterrupted())
  {
    duration -= timestamp - last_interruption_timestamp_;
  }
  return duration;
}

bool PreemptiveTask::isPreemptive() const { return true; }

ros::Time PreemptiveTask::getLastInterruptionTimestamp() const
{
  if (isInterrupted())
  {
    return last_interruption_timestamp_;
  }
  utilities::TimeIntervalPtr last_interval(interruptions_.back());
  return !interruptions_.empty() ? last_interval->getMin() : ros::Time();
}

ros::Time PreemptiveTask::getLastResumeTimestamp() const
{
  utilities::TimeIntervalPtr last_interval(interruptions_.back());
  return !isInterrupted() && !interruptions_.empty() ? last_interval->getMax()
                                                     : ros::Time();
}

bool PreemptiveTask::isInterrupted() const
{
  return Task::isRunning() && !last_interruption_timestamp_.isZero();
}

bool PreemptiveTask::isRunning() const
{
  return Task::isRunning() && last_interruption_timestamp_.isZero();
}

ruler_msgs::Task PreemptiveTask::toMsg() const
{
  ruler_msgs::Task msg(Task::toMsg());
  for (interruptions_const_iterator it(interruptions_.begin());
       it != interruptions_.end(); it++)
  {
    utilities::TimeIntervalPtr interruption(*it);
    msg.min_interruption_timestamps.push_back(interruption->getMin());
    msg.max_interruption_timestamps.push_back(interruption->getMax());
  }
}
}
