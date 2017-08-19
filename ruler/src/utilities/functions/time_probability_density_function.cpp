/**
 * This source file implements the TimeProbabilityDensityFunction class, which
 *is
 *based on the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/exception.h"
#include "utilities/functions/time_probability_density_function.h"

namespace utilities
{
namespace functions
{
TimeProbabilityDensityFunction::TimeProbabilityDensityFunction(
    const utilities::Interval<ros::Time> interval)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(
          interval.getMin().toSec(), interval.getMax().toSec())
{
}

TimeProbabilityDensityFunction::TimeProbabilityDensityFunction(
    const ros::Time& min_timestamp, const ros::Time& max_timestamp)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(
          min_timestamp.toSec(), max_timestamp.toSec())
{
}

TimeProbabilityDensityFunction::TimeProbabilityDensityFunction(
    const ros::Time& mu, const ros::Duration& sigma)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(mu.toSec(),
                                                             sigma.toSec())
{
}

TimeProbabilityDensityFunction::TimeProbabilityDensityFunction(
    const TimeProbabilityDensityFunction& pdf)
    : ProbabilityDensityFunction::ProbabilityDensityFunction(pdf)
{
}

TimeProbabilityDensityFunction::~TimeProbabilityDensityFunction() {}

double TimeProbabilityDensityFunction::probability(const ros::Time& t)
{
  return ProbabilityDensityFunction::probability(t.toSec());
}
}
}
