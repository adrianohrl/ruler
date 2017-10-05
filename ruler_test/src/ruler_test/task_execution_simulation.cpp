/**
 *  This source file implements the TaskExecutionSimulation class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/task_execution_simulation.h"

namespace ruler_test
{

TaskExecutionSimulation::TaskExecutionSimulation(ruler::Task* task)
    : Simulation::Simulation(), task_(task), interruption_pdf_(NULL),
      resuming_pdf_(NULL)

{
  start_pdf_ = new utilities::functions::TimeProbabilityDensityFunction(
      task->getStartTimestampBounds());
  end_pdf_ = new utilities::functions::TimeProbabilityDensityFunction(
      task->getEndTimestampBounds());
}

TaskExecutionSimulation::~TaskExecutionSimulation()
{
  if (start_pdf_)
  {
    delete start_pdf_;
    start_pdf_ = NULL;
  }
  if (interruption_pdf_)
  {
    delete interruption_pdf_;
    interruption_pdf_ = NULL;
  }
  if (resuming_pdf_)
  {
    delete resuming_pdf_;
    resuming_pdf_ = NULL;
  }
  if (end_pdf_)
  {
    delete end_pdf_;
    end_pdf_ = NULL;
  }
}

void TaskExecutionSimulation::update(ros::Time timestamp)
{
  if (!hasStarted(timestamp))
  {
    if (mayStart(timestamp))
    {
      start(timestamp);
    }
  }
  else if (!hasFinished(timestamp))
  {
    if (mayFinish(timestamp))
    {
      finish(timestamp);
    }
  }
}

void TaskExecutionSimulation::start(ros::Time timstamp) {}

void TaskExecutionSimulation::finish(ros::Time timstamp) {}
}
