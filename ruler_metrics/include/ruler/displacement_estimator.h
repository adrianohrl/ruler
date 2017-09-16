/**
 *  This header file defines the DisplacementEstimator class, which is
 *based on the MetricsEstimator abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_METRICS_DISPLACEMENT_ESTIMATOR_H_
#define _RULER_METRICS_DISPLACEMENT_ESTIMATOR_H_

#include <ruler/metrics_estimator.h>

namespace ruler
{
class DisplacementEstimator : public MetricsEstimator
{
public:
  DisplacementEstimator();
  virtual ~DisplacementEstimator();
  virtual void initialize(const RobotPtr& robot);
  virtual double calculate(const Task& task) const;
};
}

#endif // _RULER_METRICS_DISPLACEMENT_ESTIMATOR_H_
