/**
 *  This source file implements the BatteryConsumptionEstimator class, which is
 *based on the MetricsEstimator abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/battery_consumption_estimator.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ruler::BatteryConsumptionEstimator,
                       ruler::MetricsEstimator)

namespace ruler
{
BatteryConsumptionEstimator::BatteryConsumptionEstimator()
    : MetricsEstimator::MetricsEstimator()
{
}

BatteryConsumptionEstimator::~BatteryConsumptionEstimator() {}

void BatteryConsumptionEstimator::initialize(const RobotPtr& robot)
{
  MetricsEstimator::initialize(robot);
}

double BatteryConsumptionEstimator::calculate(const Task& task) const
{
  if (!isInitialized())
  {
    throw utilities::Exception(
        "This battery consumption estimator must be initialized fistly.");
  }
  return 1.0;
}
}
