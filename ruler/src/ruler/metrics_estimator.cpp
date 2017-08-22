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
MetricsEstimator::MetricsEstimator() : robot_(NULL) {}

MetricsEstimator::~MetricsEstimator() { robot_ = NULL; }

void MetricsEstimator::initialize(Robot* robot)
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

std::list<ResourceInterface*> MetricsEstimator::getResources() const
{
  return robot_->getResources();
}

bool MetricsEstimator::isInitialized() const { return robot_; }
}
