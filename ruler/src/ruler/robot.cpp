/**
 *  This source file implements the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/robot.h"

namespace ruler
{

std::list<ResourceInterface*> Robot::getResources() const
{
  return resources_;
}

}
