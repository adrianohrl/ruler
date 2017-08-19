/**
 *  This header file defines the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_ROBOT_H_
#define _RULER_ROBOT_H_

#include "ruler/resource_interface.h"

namespace ruler
{
class Robot
{
public:
  std::list<ResourceInterface*> getResources() const;

private:
  std::list<ResourceInterface*> resources_;
};
}

#endif // _RULER_ROBOT_H_
