#include "ruler/task.h"

namespace ruler
{

Task::Task(std::string id, std::string name, std::string description,
           bool preemptive)
    : id_(id), name_(name), description_(description), preemptive_(preemptive)
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

void Task::start() { start_time_ = ros::Time::now(); }

void Task::interrupt() {}

void Task::resume() {}

void Task::finish() { end_time_ = ros::Time::now(); }

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

const char* Task::c_str() const { return str().c_str(); }

bool Task::operator==(const Task& task) const { return id_ == task.id_; }

bool Task::operator!=(const Task& task) const { return id_ != task.id_; }
}
