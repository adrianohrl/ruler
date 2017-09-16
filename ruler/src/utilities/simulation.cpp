/**
 *  This source file implements the Simulation abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/simulation.h"

namespace utilities
{
Simulation::Simulation()
  : start_timestamp_(ros::Time::now())
{
}

Simulation::Simulation(const Simulation& simulation)
  : start_timestamp_(simulation.start_timestamp_)
{
}

Simulation::~Simulation() {}

const char *Simulation::c_str() const
{
  return str().c_str();
}

std::ostream &operator<<(std::ostream &out, const Simulation &simulation)
{
  out << simulation.str();
  return out;
}
}
