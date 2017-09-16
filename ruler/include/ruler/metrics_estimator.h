/**
 *  This header file defines the MetricsEstimator abstract class, which is a
 *pluginlib base class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_METRICS_ESTIMATOR_H_
#define _RULER_METRICS_ESTIMATOR_H_

#include "ruler/robot.h"
#include "ruler/task.h"
#include <ruler_msgs/Task.h>
#include "utilities/exception.h"

namespace ruler
{
class MetricsEstimator
{
public:
  virtual ~MetricsEstimator();
  virtual void initialize(const RobotPtr& robot);
  double calculate(const ruler_msgs::Task& msg) const;
  virtual double calculate(const Task& task) const = 0;
  virtual void shutdown();

protected:
  MetricsEstimator();
  std::list<ResourceInterfacePtr> getResources() const;
  bool isInitialized() const;

private:
  RobotPtr robot_;

};

typedef boost::shared_ptr<MetricsEstimator> MetricsEstimatorPtr;
typedef boost::shared_ptr<MetricsEstimator const> MetricsEstimatorConstPtr;
}

#endif // _RULER_METRICS_ESTIMATOR_H_
