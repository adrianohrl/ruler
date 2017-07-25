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
#include "ruler/task.h"
#include "ruler/event.h"
#include "utilities/function.h"
#include "utilities/observer.h"

namespace ruler
{
class TaskFunction : public utilities::Observer<Event>
{
public:
  TaskFunction(Task* task, utilities::Function* quantity_function);
  virtual ~TaskFunction();
  double estimate(ros::Time t) const;
  virtual void update(Event* notification);
  virtual void update(const Event& notification);

private:
  Task* task_;
  std::list<Event*> events_;
  utilities::Function* quantity_;
};
}

#endif // _RULER_TASK_FUNCTION_H_
