#include <cmath>
#include "ruler/task_simulation.h"

namespace ruler
{
TaskSimulation::TaskSimulation(
    const TaskPtr& task,
    const utilities::ContinuousNoisySignalPtr& expected_sample_time)
    : task_(task), task_start_timestamp_(task_->getExpectedStart()->random()),
      expected_duration_(task->getExpectedDuration()->random()),
      progress_level_(0.0)
{
  double sample_time(expected_sample_time->random());
  double mean(sample_time / expected_duration_.toSec());
  double standard_deviation(0.0);
  standard_deviation += pow(expected_sample_time->getStandardDeviation() /
                                expected_duration_.toSec(),
                            2);
  standard_deviation +=
      pow(-sample_time * task->getExpectedDuration()->getStandardDeviation() /
              pow(expected_duration_.toSec(), 2),
          2);
  standard_deviation = sqrt(standard_deviation);
  progress_rate_.reset(
      new utilities::ContinuousNoisySignal(mean, standard_deviation));
}

TaskSimulation::TaskSimulation(const TaskSimulation& simulation)
    : task_(simulation.task_),
      task_start_timestamp_(simulation.task_start_timestamp_),
      progress_level_(simulation.progress_level_),
      progress_rate_(simulation.progress_rate_)
{
}

TaskSimulation::~TaskSimulation() {}

void TaskSimulation::update(const ros::Time& timestamp)
{
  if (!task_->hasStarted())
  {
    task_->start();
  }
  if (task_->isRunning())
  {
    progress_level_ += progress_rate_->random();
  }
}

std::string TaskSimulation::str() const
{
  ros::Time timestamp(ros::Time::now());
  std::stringstream ss;
  ss << "[TaskSimulation] ";
  ss << "elapsed duration: " << (timestamp - start_timestamp_).toSec()
     << " [s], ";
  ros::Duration elapsed_duration;
  if (task_->hasStarted())
  {
    elapsed_duration = timestamp - task_->getStartTimestamp();
  }
  ss << "task: elapsed duration: " << elapsed_duration.toSec() << " [s], ";
  ros::Duration remaining_duration(expected_duration_ - elapsed_duration);
  ss << "remaining duration: " << remaining_duration.toSec() << " [s], ";
  ss << "expected duration: " << expected_duration_.toSec() << " [s], ";
  std::string status(!task_->hasStarted()
                         ? "NOT_STARTED"
                         : (!task_->hasFinished()) ? "STARTED" : "FINISHED");
  ss << "status: " << status << ", ";
  ss << "progress: " << progress_level_ * 100 << " [%]";
  return ss.str();
}
}
