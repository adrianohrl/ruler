/**
 *  This source file tests the main utilities and the ruler namespace classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <map>
#include <vector>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "ruler/ruler.h"
#include "utilities/utilities.h"

double tolerance(1e-4);
std::vector<double> d;
utilities::functions::ContinuousStepFunction* continuous_step;
utilities::functions::ContinuousLinearFunction* continuous_linear;
utilities::functions::ContinuousExponentialFunction* continuous_exponential;
utilities::functions::DiscreteStepFunction* discrete_step;
utilities::functions::DiscreteLinearFunction* discrete_linear;
utilities::functions::DiscreteExponentialFunction* discrete_exponential;
utilities::functions::UnaryPulseFunction* unary_pulse;
utilities::functions::UnaryStepFunction* unary_step;
std::map<double, double> q_step_asc;
std::map<double, double> q_step_des;
std::map<double, double> q_linear_asc;
std::map<double, double> q_linear_des;
std::map<double, double> q_exponential_asc;
std::map<double, double> q_exponential_des;

TEST(Functions, continuous_step)
{
  continuous_step->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_step_asc[d[i]] - continuous_step->getValue(d[i])));
  }
  continuous_step->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_step_des[d[i]] - continuous_step->getValue(d[i])));
  }
}

TEST(Functions, continuous_linear)
{
  continuous_linear->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_linear_asc[d[i]] - continuous_linear->getValue(d[i])));
  }
  continuous_linear->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_linear_des[d[i]] - continuous_linear->getValue(d[i])));
  }
}

TEST(Functions, continuous_exponential)
{
  continuous_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_exponential_asc[d[i]] -
                              continuous_exponential->getValue(d[i])));
  }
  continuous_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_exponential_des[d[i]] -
                              continuous_exponential->getValue(d[i])));
  }
}

TEST(Functions, discrete_step)
{
  discrete_step->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_step_asc[d[i]]), discrete_step->getValue(d[i]));
  }
  discrete_step->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_step_des[d[i]]), discrete_step->getValue(d[i]));
  }
}

TEST(Functions, discrete_linear)
{
  discrete_linear->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_linear_asc[d[i]]), discrete_linear->getValue(d[i]));
  }
  discrete_linear->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_linear_des[d[i]]), discrete_linear->getValue(d[i]));
  }
}

TEST(Functions, discrete_exponential)
{
  discrete_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_exponential_asc[d[i]]),
              discrete_exponential->getValue(d[i]));
  }
  discrete_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_EQ(round(q_exponential_des[d[i]]),
              discrete_exponential->getValue(d[i]));
  }
}

TEST(Functions, unary_step)
{
  unary_step->setAscending(true);
  EXPECT_FALSE(unary_step->getValue(0.0));
  EXPECT_FALSE(unary_step->getValue(0.25));
  EXPECT_TRUE(unary_step->getValue(0.5));
  EXPECT_TRUE(unary_step->getValue(1.0));
  unary_step->setAscending(false);
  EXPECT_TRUE(unary_step->getValue(0.0));
  EXPECT_TRUE(unary_step->getValue(0.25));
  EXPECT_FALSE(unary_step->getValue(0.5));
  EXPECT_FALSE(unary_step->getValue(1.0));
}

TEST(Functions, unary_pulse)
{
  unary_pulse->setAscending(true);
  EXPECT_FALSE(unary_pulse->getValue(0.0));
  EXPECT_FALSE(unary_pulse->getValue(1.0));
  EXPECT_TRUE(unary_pulse->getValue(1.25));
  EXPECT_TRUE(unary_pulse->getValue(1.5));
  EXPECT_FALSE(unary_pulse->getValue(1.75));
  EXPECT_FALSE(unary_pulse->getValue(2.0));
  unary_pulse->setAscending(false);
  EXPECT_TRUE(unary_pulse->getValue(0.0));
  EXPECT_TRUE(unary_pulse->getValue(1.0));
  EXPECT_FALSE(unary_pulse->getValue(1.25));
  EXPECT_FALSE(unary_pulse->getValue(1.5));
  EXPECT_TRUE(unary_pulse->getValue(1.75));
  EXPECT_TRUE(unary_pulse->getValue(2.0));
}

TEST(Task, start)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  task->interrupt();
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  task->resume();
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  task->finish();
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Task, interrupt)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  try
  {
    task->interrupt();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  task->interrupt();
  try
  {
    task->interrupt();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  task->finish();
  try
  {
    task->interrupt();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Task, resume)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  task->interrupt();
  task->resume();
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  task->finish();
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Task, finish)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  try
  {
    task->finish();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  task->finish();
  try
  {
    task->finish();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Profiles, continuous)
{
  ruler::Profile<utilities::ContinuousSignalType>* profile;
  try
  {
    profile = new ruler::Profile<utilities::ContinuousSignalType>(10.3, 12.1);
    delete profile;
    profile = NULL;
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  double c(1000), l0(300);
  ruler::ContinuousConsumableResource* resource =
      new ruler::ContinuousConsumableResource("r", "resource", c, l0);
  task->addResource(resource);
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  profile = new ruler::Profile<utilities::ContinuousSignalType>(c, l0);
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::ContinuousSignalType>(
          task, continuous_step));
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::ContinuousSignalType>(
          task, continuous_linear));
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::ContinuousSignalType>(
          task, continuous_exponential));
  continuous_step->setAscending(true);
  continuous_linear->setAscending(true);
  continuous_exponential->setAscending(true);
  utilities::ContinuousSignalType el, l;
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(true);
  continuous_linear->setAscending(true);
  continuous_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(true);
  continuous_linear->setAscending(false);
  continuous_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(true);
  continuous_linear->setAscending(false);
  continuous_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(false);
  continuous_linear->setAscending(true);
  continuous_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(false);
  continuous_linear->setAscending(true);
  continuous_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(false);
  continuous_linear->setAscending(false);
  continuous_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  continuous_step->setAscending(false);
  continuous_linear->setAscending(false);
  continuous_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(l0 + el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  delete profile;
  profile = NULL;
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Profiles, discrete)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  long c(1000), l0(300);
  ruler::DiscreteConsumableResource* resource =
      new ruler::DiscreteConsumableResource("r", "resource", c, l0);
  task->addResource(resource);
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  ruler::Profile<utilities::DiscreteSignalType>* profile =
      new ruler::Profile<utilities::DiscreteSignalType>(c, l0);
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::DiscreteSignalType>(task,
                                                             discrete_step));
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::DiscreteSignalType>(task,
                                                             discrete_linear));
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::DiscreteSignalType>(
          task, discrete_exponential));
  discrete_step->setAscending(true);
  discrete_linear->setAscending(true);
  discrete_exponential->setAscending(true);
  utilities::DiscreteSignalType el, l;
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_asc[d[i]]) + round(q_linear_asc[d[i]]) +
         round(q_exponential_asc[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(true);
  discrete_linear->setAscending(true);
  discrete_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_asc[d[i]]) + round(q_linear_asc[d[i]]) +
         round(q_exponential_des[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(true);
  discrete_linear->setAscending(false);
  discrete_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_asc[d[i]]) + round(q_linear_des[d[i]]) +
         round(q_exponential_asc[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(true);
  discrete_linear->setAscending(false);
  discrete_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_asc[d[i]]) + round(q_linear_des[d[i]]) +
         round(q_exponential_des[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(false);
  discrete_linear->setAscending(true);
  discrete_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_des[d[i]]) + round(q_linear_asc[d[i]]) +
         round(q_exponential_asc[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(false);
  discrete_linear->setAscending(true);
  discrete_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_des[d[i]]) + round(q_linear_asc[d[i]]) +
         round(q_exponential_des[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(false);
  discrete_linear->setAscending(false);
  discrete_exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_des[d[i]]) + round(q_linear_des[d[i]]) +
         round(q_exponential_asc[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  discrete_step->setAscending(false);
  discrete_linear->setAscending(false);
  discrete_exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = round(q_step_des[d[i]]) + round(q_linear_des[d[i]]) +
         round(q_exponential_des[d[i]]);
    l = profile->getLevel(timestamp + ros::Duration(d[i]));
    EXPECT_EQ(l0 + el, l);
    EXPECT_GE(l, 0l);
    EXPECT_LE(l, c);
  }
  delete profile;
  profile = NULL;
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(Profiles, unary)
{
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  ruler::Profile<utilities::UnarySignalType>* profile =
      new ruler::Profile<utilities::UnarySignalType>(true, false);
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::UnarySignalType>(task, unary_pulse));
  profile->addTaskFunction(
      new ruler::TaskFunction<utilities::UnarySignalType>(task, unary_step));
  unary_pulse->setAscending(true);
  unary_step->setAscending(true);
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(0.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.75)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.75)));
  unary_pulse->setAscending(true);
  unary_step->setAscending(false);
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.25)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(0.5)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(0.75)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.5)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(1.75)));
  unary_pulse->setAscending(false);
  unary_step->setAscending(true);
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.75)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.75)));
  unary_pulse->setAscending(false);
  unary_step->setAscending(false);
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.25)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(0.75)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(1.25)));
  EXPECT_FALSE(profile->getLevel(timestamp + ros::Duration(1.5)));
  EXPECT_TRUE(profile->getLevel(timestamp + ros::Duration(1.75)));
  delete profile;
  profile = NULL;
  delete task;
  task = NULL;
  delete resource;
  resource = NULL;
}

TEST(ResourceReservationRequests, consumableProduction)
{
  ruler::ContinuousConsumableResource* r1 =
      new ruler::ContinuousConsumableResource("r1", "resource 1", 10, 4);
  ruler::DiscreteConsumableResource* r2 =
      new ruler::DiscreteConsumableResource("r2", "resource 2", 5, 3);
  ruler::UnaryConsumableResource* r3 =
      new ruler::UnaryConsumableResource("r3", "resource 3", false);
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::ContinuousSignalType>(
          task, r1, new utilities::functions::ContinuousStepFunction(1.0, 3.0),
          false));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::DiscreteSignalType>(
          task, r2, new utilities::functions::DiscreteStepFunction(2.0, 2.0), false));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::UnarySignalType>(task, r3, 6.0, false));
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(0.0))));
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(1.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(1.01))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(2.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(3.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(4.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(5.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(6.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(7.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(8.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(9.0))));
  EXPECT_GE(tolerance,
            fabs(4 + 3 - r1->getLevel(timestamp + ros::Duration(10.0))));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(2.01)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_EQ(3 + 2, r2->getLevel(timestamp + ros::Duration(10.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(6.01)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(10.0)));
  delete task;
  task = NULL;
  delete r1;
  r1 = NULL;
  delete r2;
  r2 = NULL;
  delete r3;
  r3 = NULL;
}

TEST(ResourceReservationRequests, consumableConsumption)
{
  ruler::ContinuousConsumableResource* r1 =
      new ruler::ContinuousConsumableResource("r1", "resource 1", 10, 4);
  ruler::DiscreteConsumableResource* r2 =
      new ruler::DiscreteConsumableResource("r2", "resource 2", 5, 3);
  ruler::UnaryConsumableResource* r3 =
      new ruler::UnaryConsumableResource("r3", "resource 3", true);
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::ContinuousSignalType>(
          task, r1, new utilities::functions::ContinuousStepFunction(1.0, 3.0)));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::DiscreteSignalType>(
          task, r2, new utilities::functions::DiscreteStepFunction(2.0, 2.0)));
  task->addResourceReservationRequest(
      new ruler::ConsumableResourceReservationRequest<
          utilities::UnarySignalType>(task, r3, 6.0));
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(0.0))));
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(1.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(1.01))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(2.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(3.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(4.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(5.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(6.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(7.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(8.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(9.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(10.0))));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(2.01)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(10.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(6.01)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(10.0)));
  delete task;
  task = NULL;
  delete r1;
  r1 = NULL;
  delete r2;
  r2 = NULL;
  delete r3;
  r3 = NULL;
}

TEST(ResourceReservationRequests, reusable)
{
  ruler::ContinuousReusableResource* r1 =
      new ruler::ContinuousReusableResource("r1", "resource 1", 10, 4);
  ruler::DiscreteReusableResource* r2 =
      new ruler::DiscreteReusableResource("r2", "resource 2", 5, 3);
  ruler::UnaryReusableResource* r3 =
      new ruler::UnaryReusableResource("r3", "resource 3", true);
  ruler::Task* task = new ruler::Task("t", "task", ros::Duration(10));
  task->addResourceReservationRequest(
      new ruler::ReusableResourceReservationRequest<
          utilities::ContinuousSignalType>(task, r1, 3.0, 1.0));
  task->addResourceReservationRequest(
      new ruler::ReusableResourceReservationRequest<
          utilities::DiscreteSignalType>(task, r2, 2l, 2.0));
  task->addResourceReservationRequest(
      new ruler::ReusableResourceReservationRequest<utilities::UnarySignalType>(
          task, r3, 6.0));
  task->start();
  ros::Time timestamp(task->getStartTimestamp());
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(0.0))));
  EXPECT_GE(tolerance, fabs(4 - r1->getLevel(timestamp + ros::Duration(1.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(1.01))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(2.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(3.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(4.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(5.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(6.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(7.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(8.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(9.0))));
  EXPECT_GE(tolerance,
            fabs(4 - 3 - r1->getLevel(timestamp + ros::Duration(10.0))));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_EQ(3, r2->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(2.01)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_EQ(3 - 2, r2->getLevel(timestamp + ros::Duration(10.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(0.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(1.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(2.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(3.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(4.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(5.0)));
  EXPECT_TRUE(r3->getLevel(timestamp + ros::Duration(6.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(6.01)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(7.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(8.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(9.0)));
  EXPECT_FALSE(r3->getLevel(timestamp + ros::Duration(10.0)));
  delete task;
  task = NULL;
  delete r1;
  r1 = NULL;
  delete r2;
  r2 = NULL;
  delete r3;
  r3 = NULL;
}

void init()
{
  d.push_back(0.0);
  d.push_back(0.5);
  d.push_back(1.5);
  d.push_back(2.0);
  d.push_back(3.5);
  d.push_back(5.0);
  d.push_back(5.5);
  d.push_back(6.0);
  // ascending step function expected quantities
  q_step_asc.insert(std::pair<double, double>(d[0], 6.1));
  q_step_asc.insert(std::pair<double, double>(d[1], 6.1));
  q_step_asc.insert(std::pair<double, double>(d[2], 6.1));
  q_step_asc.insert(std::pair<double, double>(d[3], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[4], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[5], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[6], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[7], 8.1));
  // descending step function expected quantities
  q_step_des.insert(std::pair<double, double>(d[0], 8.1));
  q_step_des.insert(std::pair<double, double>(d[1], 8.1));
  q_step_des.insert(std::pair<double, double>(d[2], 8.1));
  q_step_des.insert(std::pair<double, double>(d[3], 6.1));
  q_step_des.insert(std::pair<double, double>(d[4], 6.1));
  q_step_des.insert(std::pair<double, double>(d[5], 6.1));
  q_step_des.insert(std::pair<double, double>(d[6], 6.1));
  q_step_des.insert(std::pair<double, double>(d[7], 6.1));
  // ascending linear function expected quantities
  q_linear_asc.insert(std::pair<double, double>(d[0], 6.1));
  q_linear_asc.insert(std::pair<double, double>(d[1], 6.1));
  q_linear_asc.insert(std::pair<double, double>(d[2], 6.1));
  q_linear_asc.insert(std::pair<double, double>(d[3], 6.35));
  q_linear_asc.insert(std::pair<double, double>(d[4], 7.1));
  q_linear_asc.insert(std::pair<double, double>(d[5], 7.85));
  q_linear_asc.insert(std::pair<double, double>(d[6], 8.1));
  q_linear_asc.insert(std::pair<double, double>(d[7], 8.1));
  // descending linear function expected quantities
  q_linear_des.insert(std::pair<double, double>(d[0], 8.1));
  q_linear_des.insert(std::pair<double, double>(d[1], 8.1));
  q_linear_des.insert(std::pair<double, double>(d[2], 8.1));
  q_linear_des.insert(std::pair<double, double>(d[3], 7.85));
  q_linear_des.insert(std::pair<double, double>(d[4], 7.1));
  q_linear_des.insert(std::pair<double, double>(d[5], 6.35));
  q_linear_des.insert(std::pair<double, double>(d[6], 6.1));
  q_linear_des.insert(std::pair<double, double>(d[7], 6.1));
  // ascending exponential function expected quantities
  q_exponential_asc.insert(std::pair<double, double>(d[0], 6.1));
  q_exponential_asc.insert(std::pair<double, double>(d[1], 6.1));
  q_exponential_asc.insert(std::pair<double, double>(d[2], 6.1));
  q_exponential_asc.insert(std::pair<double, double>(d[3], 7.02948));
  q_exponential_asc.insert(std::pair<double, double>(d[4], 7.93583));
  q_exponential_asc.insert(std::pair<double, double>(d[5], 8.07482));
  q_exponential_asc.insert(std::pair<double, double>(d[6], 8.08652));
  q_exponential_asc.insert(std::pair<double, double>(d[7], 8.1));
  // descending exponential function expected quantities
  q_exponential_des.insert(std::pair<double, double>(d[0], 8.1));
  q_exponential_des.insert(std::pair<double, double>(d[1], 8.1));
  q_exponential_des.insert(std::pair<double, double>(d[2], 8.1));
  q_exponential_des.insert(std::pair<double, double>(d[3], 7.17052));
  q_exponential_des.insert(std::pair<double, double>(d[4], 6.26417));
  q_exponential_des.insert(std::pair<double, double>(d[5], 6.12518));
  q_exponential_des.insert(std::pair<double, double>(d[6], 6.11348));
  q_exponential_des.insert(std::pair<double, double>(d[7], 6.1));
  // functions
  double d0(1.5), df(5.5), q0(6.1), qf(8.1);
  continuous_step =
      new utilities::functions::ContinuousStepFunction(d0, q0, qf);
  continuous_linear =
      new utilities::functions::ContinuousLinearFunction(d0, df, q0, qf);
  continuous_exponential =
      new utilities::functions::ContinuousExponentialFunction(d0, df, q0, qf);
  discrete_step =
      new utilities::functions::DiscreteStepFunction(d0, q0, qf);
  discrete_linear =
      new utilities::functions::DiscreteLinearFunction(d0, df, q0, qf);
  discrete_exponential =
      new utilities::functions::DiscreteExponentialFunction(d0, df, q0, qf);
  unary_pulse = new utilities::functions::UnaryPulseFunction(1.0, 1.5);
  unary_step = new utilities::functions::UnaryStepFunction(0.25);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
