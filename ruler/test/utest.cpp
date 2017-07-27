/**
 *  This source file tests the main utility classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "utilities/utilities.h"

TEST(Functions, step)
{
  utilities::Function* function = new utilities::StepFunction(1.5, 5.5, 6.1, 8.1);
  function->setAscending(true);
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  function->setAscending(false);
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  delete function;
}

TEST(Functions, linear)
{
  utilities::Function* function = new utilities::LinearFunction(1.5, 5.5, 6.1, 8.1);
  function->setAscending(true);
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(6.35 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(7.1 - function->getValue(3.5))); // for (d = 3.5 [s])
  EXPECT_GE(1e-3, fabs(7.85 - function->getValue(5.0))); // for (d = 5.0 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  function->setAscending(false);
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(7.85 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(7.1 - function->getValue(3.5))); // for (d = 3.5 [s])
  EXPECT_GE(1e-3, fabs(6.35 - function->getValue(5.0))); // for (d = 5.0 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  delete function;
}

TEST(Functions, exponential)
{
  utilities::Function* function = new utilities::ExponentialFunction(1.5, 5.5, 6.1, 8.1);
  function->setAscending(true);
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(7.02948 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(7.93583 - function->getValue(3.5))); // for (d = 3.5 [s])
  EXPECT_GE(1e-3, fabs(8.07482 - function->getValue(5.0))); // for (d = 5.0 [s])
  EXPECT_GE(1e-3, fabs(8.08652 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  function->setAscending(false);
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0))); // for (d = 0 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(0.5))); // for (d = 0.5 [s])
  EXPECT_GE(1e-3, fabs(8.1 - function->getValue(1.5))); // for (d = 1.5 [s])
  EXPECT_GE(1e-3, fabs(7.17052 - function->getValue(2.0))); // for (d = 2.0 [s])
  EXPECT_GE(1e-3, fabs(6.26417 - function->getValue(3.5))); // for (d = 3.5 [s])
  EXPECT_GE(1e-3, fabs(6.12518 - function->getValue(5.0))); // for (d = 5.0 [s])
  EXPECT_GE(1e-3, fabs(6.11348 - function->getValue(5.5))); // for (d = 5.5 [s])
  EXPECT_GE(1e-3, fabs(6.1 - function->getValue(6.0))); // for (d = 6.0 [s])
  delete function;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
