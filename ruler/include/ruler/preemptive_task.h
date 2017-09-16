#ifndef _RULER_PREEMPTIVE_TASK_H_
#define _RULER_PREEMPTIVE_TASK_H_

#include "ruler/task.h"

namespace ruler
{
class PreemptiveTask : public Task
{
public:
  PreemptiveTask(const std::string& id, const std::string& name);
  virtual ~PreemptiveTask();
  void interrupt(const ros::Time& timestamp = ros::Time::now());
  void resume(const ros::Time& timestamp = ros::Time::now());
  virtual double getDuration(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Time getLastInterruptionTimestamp() const;
  ros::Time getLastResumeTimestamp() const;
  bool isInterrupted() const;
  virtual bool isRunning() const;

private:
  ros::Time last_interruption_timestamp_;
  std::list<TimeIntervalPtr> interruption_intervals_;
};

typedef boost::shared_ptr<PreemptiveTask> PreemptiveTaskPtr;
typedef boost::shared_ptr<PreemptiveTask const> PreemptiveTaskConstPtr;
}

#endif // _RULER_PREEMPTIVE_TASK_H_
