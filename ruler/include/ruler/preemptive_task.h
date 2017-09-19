#ifndef _RULER_PREEMPTIVE_TASK_H_
#define _RULER_PREEMPTIVE_TASK_H_

#include "ruler/task.h"

namespace ruler
{
class PreemptiveTask : public Task
{
public:
  PreemptiveTask(const std::string& id, const std::string& name,
       const utilities::NoisyTimePtr& expected_start,
       const utilities::NoisyTimePtr& expected_end,
       const std::list<geometry_msgs::Pose>& waypoints =
           std::list<geometry_msgs::Pose>());
  PreemptiveTask(const ruler_msgs::Task& msg);
  PreemptiveTask(const PreemptiveTask& task);
  virtual ~PreemptiveTask();
  void interrupt(const ros::Time& timestamp = ros::Time::now());
  void resume(const ros::Time& timestamp = ros::Time::now());
  virtual void finish(const ros::Time& timestamp = ros::Time::now());
  virtual ros::Duration getDuration(const ros::Time& timestamp = ros::Time::now()) const;
  virtual bool isPreemptive() const;
  ros::Time getLastInterruptionTimestamp() const;
  ros::Time getLastResumeTimestamp() const;
  bool isInterrupted() const;
  virtual bool isRunning() const;
  virtual ruler_msgs::Task toMsg() const;

private:
  typedef std::list<utilities::TimeIntervalPtr>::iterator interruptions_iterator;
  typedef std::list<utilities::TimeIntervalPtr>::const_iterator interruptions_const_iterator;
  ros::Time last_interruption_timestamp_;
  std::list<utilities::TimeIntervalPtr> interruptions_;
};

typedef boost::shared_ptr<PreemptiveTask> PreemptiveTaskPtr;
typedef boost::shared_ptr<PreemptiveTask const> PreemptiveTaskConstPtr;
}

#endif // _RULER_PREEMPTIVE_TASK_H_
