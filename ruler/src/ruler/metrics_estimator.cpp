/**
 *  This source file implements the MetricsEstimator abstract class, which is a
 *pluginlib base class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/metrics_estimator.h"

namespace ruler
{
MetricsEstimator::MetricsEstimator() {}

MetricsEstimator::~MetricsEstimator() {}

void MetricsEstimator::initialize(const RobotPtr& robot)
{
  if (!robot)
  {
    throw utilities::Exception("Robot must not be NULL.");
  }
  robot_ = robot;
}

double MetricsEstimator::calculate(const ruler_msgs::Task& msg) const
{
  return calculate(Task(msg));
}

void MetricsEstimator::shutdown() {}

std::list<ResourceInterfacePtr> MetricsEstimator::getResources() const
{
  return robot_->getResources();
}

bool MetricsEstimator::isInitialized() const { return robot_.get(); }
}
