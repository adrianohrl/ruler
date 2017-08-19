/**
 *  This header file defines the Simulation abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIMULATION_H_
#define _UTILITIES_SIMULATION_H_

#include <ros/time.h>
#include "utilities/functions/time_probability_density_function.h"

namespace utilities
{
class Simulation
{
public:
  virtual ~Simulation();
  virtual void update(ros::Time timestamp = ros::Time::now());

protected:
  Simulation(
      utilities::functions::TimeProbabilityDensityFunction* start_pdf,
      utilities::functions::TimeProbabilityDensityFunction* end_pdf);
  Simulation(const Simulation& simulation);
  virtual bool mayStart(ros::Time timestamp) const;
  virtual bool mayFinish(ros::Time timestamp) const;

private:
  double start_probability_;
  double end_probability_;
  utilities::functions::TimeProbabilityDensityFunction* start_pdf_;
  utilities::functions::TimeProbabilityDensityFunction* end_pdf_;
  virtual void start(ros::Time timestamp) = 0;
  virtual void finish(ros::Time timestamp) = 0;
  virtual bool hasStarted(ros::Time timestamp) const = 0;
  virtual bool hasFinished(ros::Time timestamp) const = 0;
};
}

#endif // _UTILITIES_SIMULATION_H_
