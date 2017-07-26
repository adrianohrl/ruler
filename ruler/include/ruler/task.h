/**
 *  This header file defines the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_H_
#define _RULER_TASK_H_

#include <list>
#include <string>
#include <ros/time.h>
#include "ruler/event.h"
#include "ruler/resource.h"
#include "utilities/interval.h"
#include "utilities/subject.h"

namespace ruler
{
class Task : public utilities::Subject<Event>
{
public:
  Task(std::string id, std::string name, std::string description = "",
       bool preemptive = false);
  Task(const Task& task);
  virtual ~Task();
  void start();
  void interrupt();
  void resume();
  void finish();
  template <typename T> void addResource(Resource<T>* resource);
  template <typename T> void removeResource(const Resource<T>& resource);
  void clearResources();
  double getDuration(ros::Time t = ros::Time::now()) const;
  std::string getId() const;
  std::string getName() const;
  std::string getDescription() const;
  bool isPreemptive() const;
  ros::Time getStartTimestamp() const;
  ros::Time getEndTimestamp() const;
  void setDescription(std::string description);
  std::string str() const;
  bool operator==(const Task& task) const;
  bool operator!=(const Task& task) const;

private:
  std::string id_;
  std::string name_;
  std::string description_;
  bool preemptive_;
  ros::Time start_timestamp_;
  ros::Time last_interruption_timestamp_;
  ros::Time end_timestamp_;
  utilities::Interval<ros::Time>* start_timestamp_bounds_;
  utilities::Interval<ros::Time>* end_timestamp_bounds_;
  std::list<utilities::Interval<ros::Time>*> interruption_intervals_;
};

template <typename T> void Task::addResource(Resource<T>* resource)
{
  utilities::Subject<Event>::registerObserver(resource);
}

template <typename T> void Task::removeResource(const Resource<T>& resource)
{
  utilities::Subject<Event>::unregisterObserver(resource);
}
}

#endif // _RULER_TASK_H_
