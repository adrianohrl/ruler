#ifndef _RULER_SIMULATED_TASK_H_
#define _RULER_SIMULATED_TASK_H_

#include "ruler/task.h"
#include "utilities/continuous_noisy_signal.h"
#include "utilities/noisy_duration.h"
#include "utilities/noisy_time.h"
#include "utilities/simulation.h"

namespace ruler
{
class TaskSimulation : public utilities::Simulation
{
public:
  TaskSimulation(
      const TaskPtr& task,
      const utilities::ContinuousNoisySignalPtr& expected_sample_time);
  TaskSimulation(const TaskSimulation& simulation);
  virtual ~TaskSimulation();
  virtual void abort(const ros::Time& timestamp = ros::Time::now());
  virtual void update(const ros::Time& timestamp = ros::Time::now());
  virtual ros::Duration getTaskElapsedDuration(
      const ros::Time& timestamp = ros::Time::now()) const;
  virtual ros::Duration getTaskRemainingDuration(
      const ros::Time& timestamp = ros::Time::now()) const;
  virtual ros::Duration getTaskExpectedDuration() const;
  TaskConstPtr getTask() const;
  virtual std::string str() const;

private:
  TaskPtr task_;
  ros::Time task_start_timestamp_;
  ros::Duration expected_duration_;
  double progress_level_;
  bool aborted_;
  utilities::ContinuousNoisySignalPtr progress_rate_;
};

typedef boost::shared_ptr<TaskSimulation> TaskSimulationPtr;
typedef boost::shared_ptr<TaskSimulation const> TaskSimulationConstPtr;
}

#endif // _RULER_SIMULATED_TASK_H_
