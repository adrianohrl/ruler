/**
 * This header file defines the TimeProbabilityDensityFunction class, which is
 *based on the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_TIME_PROBABILITY_DENSITY_FUNCTION_H_
#define _UTILITIES_TIME_PROBABILITY_DENSITY_FUNCTION_H_

#include <ros/time.h>
#include "utilities/interval.h"
#include "utilities/functions/probability_density_function.h"

namespace utilities
{
namespace functions
{
class TimeProbabilityDensityFunction : public ProbabilityDensityFunction
{
public:
  TimeProbabilityDensityFunction(const utilities::Interval<ros::Time> interval);
  TimeProbabilityDensityFunction(const ros::Time& min_timestamp,
                                 const ros::Time& max_timestamp);
  TimeProbabilityDensityFunction(const ros::Time& mu,
                                 const ros::Duration& sigma);
  TimeProbabilityDensityFunction(const TimeProbabilityDensityFunction& pdf);
  virtual ~TimeProbabilityDensityFunction();
  double probability(const ros::Time& t = ros::Time::now());
};
}
}

#endif // _UTILITIES_TIME_PROBABILITY_DENSITY_FUNCTION_H_
