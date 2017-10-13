#include "ruler/task_simulation.h"
#include <cmath>

namespace ruler
{
TaskSimulation::TaskSimulation(
    const TaskPtr& task,
    const utilities::ContinuousNoisySignalPtr& expected_sample_time)
    : task_(task), task_start_timestamp_(task_->getExpectedStart()->random()),
      expected_duration_(task->getExpectedDuration()->random()),
      progress_level_(0.0), aborted_(false)
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

void TaskSimulation::abort(const ros::Time& timestamp)
{
  if (aborted_)
  {
    ROS_WARN_STREAM(task_->getId() << " is already aborted!!!");
    return;
  }
  aborted_ = true;
  task_->finish(timestamp);
  ROS_INFO_STREAM(task_->getId() << " has been aborted!!!");
}

void TaskSimulation::update(const ros::Time& timestamp)
{
  if (aborted_)
  {
    return;
  }
  if (timestamp <= last_update_timestamp_)
  {
    throw utilities::Exception(
        "The simulation update timestamp must be after its last one.");
  }
  if (!task_->hasStarted() && timestamp >= task_start_timestamp_)
  {
    task_->start(timestamp);
  }
  if (task_->isRunning())
  {
    progress_level_ += progress_rate_->random();
  }
  if (!task_->hasFinished() && progress_level_ >= 1.0)
  {
    progress_level_ = 1.0;
    task_->finish(timestamp);
  }
  last_update_timestamp_ = timestamp;
}

ros::Duration
TaskSimulation::getTaskElapsedDuration(const ros::Time& timestamp) const
{
  return task_->hasStarted() ? task_->getDuration(timestamp) : ros::Duration();
}

ros::Duration
TaskSimulation::getTaskRemainingDuration(const ros::Time& timestamp) const
{
  ros::Duration elapsed_duration(getTaskElapsedDuration(timestamp));
  return expected_duration_ > elapsed_duration ? elapsed_duration
                                               : ros::Duration();
}

ros::Duration TaskSimulation::getTaskExpectedDuration() const
{
  return expected_duration_;
}

TaskConstPtr TaskSimulation::getTask() const
{
  return task_;
}

std::string TaskSimulation::str() const
{
  std::stringstream ss;
  ss << "[TaskSimulation] ";
  ss << "elapsed duration: "
     << getSimulationElapsedDuration(last_update_timestamp_).toSec()
     << " [s], ";
  ss << "task: elapsed duration: "
     << getTaskElapsedDuration(last_update_timestamp_).toSec() << " [s], ";
  ss << "remaining duration: "
     << getTaskRemainingDuration(last_update_timestamp_).toSec() << " [s], ";
  ss << "expected duration: " << expected_duration_.toSec() << " [s], ";
  std::string status(!task_->hasStarted()
                         ? "NOT_STARTED"
                         : (!task_->hasFinished()) ? "STARTED" : "FINISHED");
  ss << "status: " << status << ", ";
  if (status == "STARTED")
  {
    ss << "progress: " << progress_level_ * 100 << " [%]";
  }
  return ss.str();
}
}
