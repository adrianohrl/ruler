/**
 * This header file defines the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_
#define _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_

#include <sstream>

namespace utilities
{
namespace functions
{
class ProbabilityDensityFunction
{
public:
  ProbabilityDensityFunction(double mu, double sigma);
  ProbabilityDensityFunction(double neg_n_sigma, double pos_n_sigma, double n);
  ProbabilityDensityFunction(const ProbabilityDensityFunction &pdf);
  virtual ~ProbabilityDensityFunction();
  double probability(double x);
  std::string str() const;
  const char* c_str() const;

private:
  static const double TOLERANCE = 1e-4;
  double mu_;
  double sigma_;
};
}
}

#endif // _UTILITIES_PROBABILITY_DENSITY_FUNCTION_H_
