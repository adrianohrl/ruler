/**
 *  This header file implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/task.h"

namespace ruler
{
Task::Task(std::string id, std::string name, bool preemptive)
    : utilities::Subject<TaskEvent>::Subject(id), name_(name),
      preemptive_(preemptive), start_timestamp_bounds_(NULL),
      end_timestamp_bounds_(NULL)
{
}

Task::Task(const Task& task)
    : utilities::Subject<TaskEvent>::Subject(task), name_(task.name_),
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
  std::list<utilities::Interval<ros::Time>*>::iterator it(
      interruption_intervals_.begin());
  while (it != interruption_intervals_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

void Task::start()
{
  if (!start_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has already been started.");
  }
  if (utilities::Subject<TaskEvent>::empty())
  {
    throw utilities::Exception(str() + " does not have any resource registered yet.");
  }
  start_timestamp_ = ros::Time::now();
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::STARTED));
  ROS_DEBUG_STREAM(*this << " has just started.");
}

void Task::interrupt()
{
  if (start_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has not been started yet.");
  }
  if (!end_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  if (!last_interruption_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has already been interrupted.");
  }
  last_interruption_timestamp_ = ros::Time::now();
  utilities::Subject<TaskEvent>::notify(TaskEvent(this, types::INTERRUPTED));
  ROS_DEBUG_STREAM(*this << " has just interruped.");
}

void Task::resume()
{
  if (start_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has not started yet.");
  }
  if (!end_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has already been finished.");
  }
  if (last_interruption_timestamp_.isZero())
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
  if (start_timestamp_.isZero())
  {
    throw utilities::Exception(str() + " has not started yet.");
  }
  if (!end_timestamp_.isZero())
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
  if (end_timestamp_.isZero() && !last_interruption_timestamp_.isZero())
  {
    duration -= (t - last_interruption_timestamp_).toSec();
  }
  return duration;
}

std::string Task::getName() const { return name_; }

bool Task::isPreemptive() const { return preemptive_; }

ros::Time Task::getStartTimestamp() const { return start_timestamp_; }

ros::Time Task::getEndTimestamp() const { return end_timestamp_; }
}
