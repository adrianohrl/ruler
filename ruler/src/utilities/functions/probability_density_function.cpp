/**
 * This source file implements the ProbabilityDensityFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <cmath>
#include "utilities/exception.h"
#include "utilities/functions/probability_density_function.h"

namespace utilities
{
namespace functions
{
ProbabilityDensityFunction::ProbabilityDensityFunction(double mu, double sigma)
    : mu_(mu),
      sigma_(fabs(sigma == 0.0 ? ProbabilityDensityFunction::TOLERANCE : sigma))
{
}

ProbabilityDensityFunction::ProbabilityDensityFunction(double neg_n_sigma,
                                                       double pos_n_sigma,
                                                       double n)
    : mu_((pos_n_sigma + neg_n_sigma) / 2),
      sigma_(fabs(pos_n_sigma == neg_n_sigma
                      ? ProbabilityDensityFunction::TOLERANCE
                      : pos_n_sigma - neg_n_sigma) /
             (2 * fabs(n == 0 ? ProbabilityDensityFunction::TOLERANCE : n)))
{
}

ProbabilityDensityFunction::ProbabilityDensityFunction(
    const ProbabilityDensityFunction& pdf)
    : mu_(pdf.mu_), sigma_(pdf.sigma_)
{
}

ProbabilityDensityFunction::~ProbabilityDensityFunction() {}

double ProbabilityDensityFunction::probability(double x)
{
  return 1 / (sqrt(2 * M_PI) * sigma_) * exp(-0.5 * pow((x - mu_) / sigma_, 2));
}

std::string ProbabilityDensityFunction::str() const
{
  std::stringstream ss;
  ss << "X ~ N(" << mu_ << "; " << sigma_* sigma_ << ")";
  return ss.str();
}

const char* ProbabilityDensityFunction::c_str() const { return str().c_str(); }
}
}
