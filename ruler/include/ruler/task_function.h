/**
 *  This header file defines and implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_FUNCTION_H_
#define _RULER_TASK_FUNCTION_H_

#include <list>
#include <ros/time.h>
#include "ruler/task_event.h"
#include "utilities/function.h"

namespace ruler
{
class Task;

class TaskFunction
{
public:
  TaskFunction(Task* task, utilities::Function* quantity_function);
  virtual ~TaskFunction();
  void update(const TaskEvent& notification);
  double getLevel(ros::Time t) const;
  double getLevel(double d) const;
  Task* getTask() const;

private:
  Task* task_;
  std::list<TaskEvent> events_;
  utilities::Function* quantity_function_;
};
}

#endif // _RULER_TASK_FUNCTION_H_
