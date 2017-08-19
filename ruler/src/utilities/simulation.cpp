/**
 *  This source file implements the Simulation abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/simulation.h"

namespace utilities
{
Simulation::Simulation(
    utilities::functions::TimeProbabilityDensityFunction* start_pdf,
    double start_probability,
    utilities::functions::TimeProbabilityDensityFunction* end_pdf,
    double end_probability)
    : start_pdf_(start_pdf), start_probability_(fabs(start_probability)),
      end_probability_(fabs(end_probability)), end_pdf_(end_pdf)
{
}

Simulation::Simulation(const Simulation& simulation)
    : start_pdf_(new utilities::functions::TimeProbabilityDensityFunction(
          *simulation.start_pdf_)),
      start_probability_(simulation.start_probability_),
      end_pdf_(new utilities::functions::TimeProbabilityDensityFunction(
          *simulation.end_pdf_)),
      end_probability_(simulation.end_probability_)
{
}

void Simulation::update(ros::Time timestamp)
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

bool Simulation::mayStart(ros::Time timestamp) const
{
  return start_pdf_->probability(timestamp) >= start_probability_;
}

bool Simulation::mayFinish(ros::Time timestamp) const
{
  return end_pdf_->probability(timestamp) >= end_probability_;
}
}
