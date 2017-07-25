/**
 *  This source file implements the main function that tests the ruler library.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <iostream>
#include "ruler/continuous_consumable_resource.h"
#include "ruler/continuous_reusable_resource.h"
#include "ruler/discrete_consumable_resource.h"
#include "ruler/discrete_reusable_resource.h"
#include "ruler/task.h"
#include "ruler/unary_consumable_resource.h"
#include "ruler/unary_reusable_resource.h"

int main(int argc, char** argv)
{
  std::cout << "Testing ruler library ..." << std::endl;
  ruler::ContinuousConsumableResource* r1 =
      new ruler::ContinuousConsumableResource("type1", "r1", 10.5);
  ruler::ContinuousReusableResource* r2 =
      new ruler::ContinuousReusableResource("type2", "r2", 5.6);
  ruler::DiscreteConsumableResource* r3 =
      new ruler::DiscreteConsumableResource("type3", "r3", 7);
  ruler::DiscreteReusableResource* r4 =
      new ruler::DiscreteReusableResource("type4", "r4", 5);
  ruler::UnaryConsumableResource* r5 =
      new ruler::UnaryConsumableResource("type5", "r5");
  ruler::UnaryReusableResource* r6 =
      new ruler::UnaryReusableResource("type6", "r6", false);
  ruler::Task* t1 = new ruler::Task("task1", "t1");
  ruler::Task* t2 = new ruler::Task("task2", "t2");
  delete r1;
  r1 = NULL;
  delete r2;
  r2 = NULL;
  delete r3;
  r3 = NULL;
  delete r4;
  r4 = NULL;
  delete r5;
  r5 = NULL;
  delete r6;
  r6 = NULL;
  return 0;
}
