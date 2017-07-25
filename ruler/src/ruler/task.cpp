/**
 *  This header file implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/task.h"

namespace ruler
{
Task::Task(std::string id, std::string name, std::string description,
           bool preemptive)
    : id_(id), name_(name), description_(description), preemptive_(preemptive),
      start_time_bounds_(NULL), end_time_bounds_(NULL)
{
}

Task::Task(const Task& task)
    : id_(task.id_), name_(task.name_), description_(task.description_),
      preemptive_(task.preemptive_), start_time_(task.start_time_),
      end_time_(task.end_time_), start_time_bounds_(task.start_time_bounds_),
      end_time_bounds_(task.end_time_bounds_),
      executed_intervals_(task.executed_intervals_)
{
}

Task::~Task()
{
  if (start_time_bounds_)
  {
    delete start_time_bounds_;
    start_time_bounds_ = NULL;
  }
  if (end_time_bounds_)
  {
    delete end_time_bounds_;
    end_time_bounds_ = NULL;
  }
  std::list<utilities::Interval<ros::Time>*>::iterator it(
      executed_intervals_.begin());
  while (it != executed_intervals_.end())
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
  if (!start_time_.isZero())
  {
    throw utilities::Exception(id_ + " has already started.");
  }
  // check other possible errors
  start_time_ = ros::Time::now();
  utilities::Subject<Event>::notify(Event(types::STARTED));
  ROS_INFO("%s has just started.", id_.c_str());
}

void Task::interrupt()
{
  if (start_time_.isZero())
  {
    throw utilities::Exception(id_ + " has not started yet.");
  }
  // check other possible errors
  // interrupt properly
  utilities::Subject<Event>::notify(Event(types::INTERRUPTED));
  ROS_INFO("%s has just interruped.", id_.c_str());
}

void Task::resume()
{
  if (start_time_.isZero())
  {
    throw utilities::Exception(id_ + " has not started yet.");
  }
  // check other possible errors
  // resume properly
  utilities::Subject<Event>::notify(Event(types::RESUMED));
  ROS_INFO("%s has just resumed.", id_.c_str());
}

void Task::finish()
{
  if (start_time_.isZero())
  {
    throw utilities::Exception(id_ + " has not started yet.");
  }
  // check other possible errors
  end_time_ = ros::Time::now();
  utilities::Subject<Event>::notify(Event(types::FINISHED));
  ROS_INFO("%s has just finished.", id_.c_str());
}

void Task::clearResources() { utilities::Subject<Event>::clearObservers(); }

double Task::getDuration(ros::Time t) const
{
  double duration(((t > end_time_ ? end_time_ : t) - start_time_).toSec());
  std::list<utilities::Interval<ros::Time>*>::const_iterator it(
      executed_intervals_.begin());
  while (it != executed_intervals_.end())
  {
    utilities::Interval<ros::Time>* interval = *it;
    if (t > interval->getMax())
    {
      duration -= (interval->getMax() - interval->getMin()).toSec();
    }
    else if (interval->belongs(t))
    {
      duration -= (t - interval->getMin()).toSec();
    }
    // se essa lista for ordenada, pode colocar um break aki dentro.
    it++;
  }
  return duration;
}

std::string Task::getId() const { return id_; }

std::string Task::getName() const { return name_; }

std::string Task::getDescription() const { return description_; }

bool Task::isPreemptive() const { return preemptive_; }

ros::Time Task::getStartTime() const { return start_time_; }

ros::Time Task::getEndTime() const { return end_time_; }

void Task::setDescription(std::string description)
{
  description_ = description;
}

std::string Task::str() const { return id_; }

bool Task::operator==(const Task& task) const { return id_ == task.id_; }

bool Task::operator!=(const Task& task) const { return id_ != task.id_; }
}
