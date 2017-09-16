/**
 *  This source file implements the DisplacementEstimator class, which is
 *based on the MetricsEstimator abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <pluginlib/class_list_macros.h>
#include "ruler/displacement_estimator.h"

PLUGINLIB_EXPORT_CLASS(ruler::DisplacementEstimator, ruler::MetricsEstimator)

namespace ruler
{
DisplacementEstimator::DisplacementEstimator()
    : MetricsEstimator::MetricsEstimator()
{
}

DisplacementEstimator::~DisplacementEstimator() {}

void DisplacementEstimator::initialize(const RobotPtr& robot)
{
  MetricsEstimator::initialize(robot);
}

double DisplacementEstimator::calculate(const Task& task) const
{
  if (!isInitialized())
  {
    throw utilities::Exception(
        "This displacement estimator must be initialized fistly.");
  }
  return task.getDistance();
}
}
