/**
 *  This header file defines and implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RESOURCE_CONTROL_TASK_H_
#define _RESOURCE_CONTROL_TASK_H_

#include <list>
#include <string>
#include <ros/time.h>
#include "utilities/interval.h"
#include "utilities/subject.h"
#include "ruler/event.h"

namespace ruler
{
class Task : public utilities::Subject<Event<T> >
{
public:
  Task(std::string id, std::string name, std::string description,
       bool preemptive);
  Task(const Task& task);
  virtual ~Task();
  void start();
  void interrupt();
  void resume();
  void finish();
  double getDuration(ros::Time t = ros::Time::now()) const;
  std::string getId() const;
  std::string getName() const;
  std::string getDescription() const;
  bool isPreemptive() const;
  ros::Time getStartTime() const;
  ros::Time getEndTime() const;
  void setDescription(std::string description);
  std::string str() const;
  const char* c_str() const;
  bool operator==(const Task& task) const;
  bool operator!=(const Task& task) const;

private:
  std::string id_, name_, description_;
  bool preemptive_;
  ros::Time start_time_, end_time_;
  utilities::Interval<ros::Time>* start_time_bounds_, end_time_bounds_;
  std::list<utilities::Interval<ros::Time>*> executed_intervals_;
};
}

#endif // _RESOURCE_CONTROL_TASK_H_
