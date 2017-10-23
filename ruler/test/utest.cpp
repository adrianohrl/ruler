/**
 *  This source file tests the main utilities and the ruler namespace classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/ruler.h"
#include "utilities/utilities.h"
#include <gtest/gtest.h>
#include <map>
#include <ros/ros.h>
#include <vector>

double tolerance(1e-4);
std::vector<double> d;
utilities::functions::ContinuousStepFunctionPtr continuous_step;
utilities::functions::ContinuousLinearFunctionPtr continuous_linear;
utilities::functions::ContinuousExponentialFunctionPtr continuous_exponential;
utilities::functions::DiscreteStepFunctionPtr discrete_step;
utilities::functions::DiscreteLinearFunctionPtr discrete_linear;
utilities::functions::DiscreteExponentialFunctionPtr discrete_exponential;
utilities::functions::UnaryPulseFunctionPtr unary_pulse;
utilities::functions::UnaryStepFunctionPtr unary_step;
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

TEST(Functions, step2pulse)
{
  utilities::functions::DiscreteStepFunctionPtr step(
      new utilities::functions::DiscreteStepFunction(2.0, 25l, true, true));
  utilities::functions::DiscretePulseFunctionPtr pulse(
      new utilities::functions::DiscretePulseFunction(*step, 4.0));
  // testing step
  EXPECT_EQ(0, step->getValue(0.0));
  EXPECT_EQ(0, step->getValue(1.9));
  EXPECT_EQ(0, step->getValue(2.0));
  EXPECT_EQ(25, step->getValue(2.1));
  EXPECT_EQ(25, step->getValue(3.9));
  EXPECT_EQ(25, step->getValue(4.0));
  EXPECT_EQ(25, step->getValue(4.1));
  // testing pulse
  EXPECT_EQ(0, pulse->getValue(0.0));
  EXPECT_EQ(0, pulse->getValue(1.9));
  EXPECT_EQ(0, pulse->getValue(2.0));
  EXPECT_EQ(25, pulse->getValue(2.1));
  EXPECT_EQ(25, pulse->getValue(3.9));
  EXPECT_EQ(25, pulse->getValue(4.0));
  EXPECT_EQ(0, pulse->getValue(4.1));
}

TEST(Functions, unary_sample_holder)
{
  double dt(0.25);
  utilities::functions::UnarySampleHolderPtr ush(
      new utilities::functions::UnarySampleHolder(
          "ush", ros::Duration(1.0 * dt), ros::Duration(3.5 * dt)));
  ros::Time timestamp = ush->getStartTimestamp();
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.25 * dt)));
  ush->update(timestamp + ros::Duration(0.5 * dt));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(0.75 * dt)));
  ush->update(timestamp + ros::Duration(1.0 * dt));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(1.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(1.75 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(2.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(2.25 * dt)));
  ush->update(timestamp + ros::Duration(2.5 * dt));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(2.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(2.75 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.25 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.5 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(3.75 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(4.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(5.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(5.5 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(6.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(6.5 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(7.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(7.5 * dt)));
  ush.reset(new utilities::functions::UnarySampleHolder(
      "ush", ros::Duration(3.5 * dt)));
  timestamp = ush->getStartTimestamp();
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.25 * dt)));
  ush->update(timestamp + ros::Duration(0.5 * dt));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(0.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(0.75 * dt)));
  ush->update(false, timestamp + ros::Duration(1.0 * dt));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(1.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(1.75 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(2.0 * dt)));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(2.25 * dt)));
  ush->update(true, timestamp + ros::Duration(2.5 * dt));
  EXPECT_FALSE(ush->getValue(timestamp + ros::Duration(2.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(2.75 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.25 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(3.75 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(4.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(5.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(5.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(6.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(6.5 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(7.0 * dt)));
  EXPECT_TRUE(ush->getValue(timestamp + ros::Duration(7.5 * dt)));
}

TEST(Functions, discrete_sample_holder)
{
  double dt(0.25);
  utilities::functions::DiscreteSampleHolderPtr dsh(
      new utilities::functions::DiscreteSampleHolder("dsh", 2l,
                                                     ros::Duration(3.5 * dt)));
  ros::Time timestamp = dsh->getStartTimestamp();
  EXPECT_EQ(0, dsh->getValue(timestamp + ros::Duration(0.0 * dt)));
  EXPECT_EQ(0, dsh->getValue(timestamp + ros::Duration(0.25 * dt)));
  dsh->update(timestamp + ros::Duration(0.5 * dt));
  EXPECT_EQ(0, dsh->getValue(timestamp + ros::Duration(0.5 * dt)));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(0.75 * dt)));
  dsh->update(timestamp + ros::Duration(1.0 * dt));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(1.0 * dt)));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(1.75 * dt)));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(2.0 * dt)));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(2.25 * dt)));
  dsh->update(3l, timestamp + ros::Duration(2.5 * dt));
  EXPECT_EQ(2, dsh->getValue(timestamp + ros::Duration(2.5 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(2.75 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(3.0 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(3.25 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(3.5 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(3.75 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(4.0 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(5.0 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(5.5 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(6.0 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(6.5 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(7.0 * dt)));
  EXPECT_EQ(3, dsh->getValue(timestamp + ros::Duration(7.5 * dt)));
}

TEST(Functions, continuous_sample_holder)
{
  double dt(0.25);
  utilities::functions::ContinuousSampleHolderPtr csh(
      new utilities::functions::ContinuousSampleHolder(
          "csh", 1.8, ros::Duration(3.5 * dt)));
  ros::Time timestamp = csh->getStartTimestamp();
  EXPECT_GE(tolerance,
            fabs(0.0 - csh->getValue(timestamp + ros::Duration(0.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(0.0 - csh->getValue(timestamp + ros::Duration(0.25 * dt))));
  csh->update(timestamp + ros::Duration(0.5 * dt));
  EXPECT_GE(tolerance,
            fabs(0.0 - csh->getValue(timestamp + ros::Duration(0.5 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.8 - csh->getValue(timestamp + ros::Duration(0.75 * dt))));
  csh->update(2.6, timestamp + ros::Duration(1.0 * dt));
  EXPECT_GE(tolerance,
            fabs(1.8 - csh->getValue(timestamp + ros::Duration(1.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(2.6 - csh->getValue(timestamp + ros::Duration(1.75 * dt))));
  EXPECT_GE(tolerance,
            fabs(2.6 - csh->getValue(timestamp + ros::Duration(2.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(2.6 - csh->getValue(timestamp + ros::Duration(2.25 * dt))));
  csh->update(1.3, timestamp + ros::Duration(2.5 * dt));
  EXPECT_GE(tolerance,
            fabs(2.6 - csh->getValue(timestamp + ros::Duration(2.5 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(2.75 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(3.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(3.25 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(3.5 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(3.75 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(4.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(5.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(5.5 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(6.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(6.5 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(7.0 * dt))));
  EXPECT_GE(tolerance,
            fabs(1.3 - csh->getValue(timestamp + ros::Duration(7.5 * dt))));
}

TEST(Task, start)
{
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  task->start();
  task->interrupt();
  try
  {
    task->start();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
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
  catch (const utilities::Exception& e)
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
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
}

TEST(Task, interrupt)
{
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  try
  {
    task->interrupt();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  task->start();
  task->interrupt();
  try
  {
    task->interrupt();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
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
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
}

TEST(Task, resume)
{
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  task->start();
  try
  {
    task->resume();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
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
  catch (const utilities::Exception& e)
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
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
}

TEST(Task, finish)
{
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  try
  {
    task->finish();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  task->start();
  task->finish();
  try
  {
    task->finish();
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
}

TEST(Profiles, continuous)
{
  ruler::ContinuousProfilePtr profile;
  try
  {
    profile.reset(new ruler::ContinuousProfile(10.3, 12.1));
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  double delta(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp + ros::Duration(1.0 * delta),
                               timestamp + ros::Duration(1.5 * delta)));
  utilities::NoisyTimePtr expected_end(
      new utilities::NoisyTime(timestamp + ros::Duration(5.0 * delta),
                               timestamp + ros::Duration(7.5 * delta)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  double c(1000), l0(300);
  ruler::ContinuousConsumableResourcePtr resource(
      new ruler::ContinuousConsumableResource("r", "resource", c, l0));
  task->addResource(resource);
  task->start();
  timestamp = task->getStartTimestamp();
  profile.reset(new ruler::ContinuousProfile(c, l0));
  profile->addTaskFunction(ruler::ContinuousTaskFunctionPtr(
      new ruler::ContinuousTaskFunction(resource, task, continuous_step)));
  profile->addTaskFunction(ruler::ContinuousTaskFunctionPtr(
      new ruler::ContinuousTaskFunction(resource, task, continuous_linear)));
  profile->addTaskFunction(
      ruler::ContinuousTaskFunctionPtr(new ruler::ContinuousTaskFunction(
          resource, task, continuous_exponential)));
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
}

TEST(Profiles, discrete)
{
  double delta(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp + ros::Duration(1.0 * delta),
                               timestamp + ros::Duration(1.5 * delta)));
  utilities::NoisyTimePtr expected_end(
      new utilities::NoisyTime(timestamp + ros::Duration(5.0 * delta),
                               timestamp + ros::Duration(7.5 * delta)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  long c(1000), l0(300);
  ruler::DiscreteConsumableResourcePtr resource(
      new ruler::DiscreteConsumableResource("r", "resource", c, l0));
  task->addResource(resource);
  task->start();
  timestamp = task->getStartTimestamp();
  ruler::DiscreteProfilePtr profile(new ruler::DiscreteProfile(c, l0));
  profile->addTaskFunction(ruler::DiscreteTaskFunctionPtr(
      new ruler::DiscreteTaskFunction(resource, task, discrete_step)));
  profile->addTaskFunction(ruler::DiscreteTaskFunctionPtr(
      new ruler::DiscreteTaskFunction(resource, task, discrete_linear)));
  profile->addTaskFunction(ruler::DiscreteTaskFunctionPtr(
      new ruler::DiscreteTaskFunction(resource, task, discrete_exponential)));
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
}

TEST(Profiles, unary)
{
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  task->start();
  timestamp = task->getStartTimestamp();
  ruler::UnaryProfilePtr profile(new ruler::UnaryProfile(true, false));
  profile->addTaskFunction(ruler::UnaryTaskFunctionPtr(
      new ruler::UnaryTaskFunction(resource, task, unary_pulse)));
  profile->addTaskFunction(ruler::UnaryTaskFunctionPtr(
      new ruler::UnaryTaskFunction(resource, task, unary_step)));
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
}

TEST(ResourceReservationRequests, consumableProduction)
{
  ruler::ContinuousConsumableResourcePtr r1(
      new ruler::ContinuousConsumableResource("r1", "resource 1", 10.0, 4.0));
  ruler::DiscreteConsumableResourcePtr r2(
      new ruler::DiscreteConsumableResource("r2", "resource 2", 5l, 3l));
  ruler::UnaryConsumableResourcePtr r3(
      new ruler::UnaryConsumableResource("r3", "resource 3", false));
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  task->addResourceReservationRequest(
      ruler::ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              task, r1,
              utilities::functions::ContinuousStepFunctionPtr(
                  new utilities::functions::ContinuousStepFunction(1.0, 3.0)),
              false)));
  task->addResourceReservationRequest(
      ruler::DiscreteConsumableResourceReservationRequestPtr(
          new ruler::DiscreteConsumableResourceReservationRequest(
              task, r2,
              utilities::functions::DiscreteStepFunctionPtr(
                  new utilities::functions::DiscreteStepFunction(2.0, 2.0)),
              false)));
  task->addResourceReservationRequest(
      ruler::UnaryConsumableResourceReservationRequestPtr(
          new ruler::UnaryConsumableResourceReservationRequest(task, r3, 6.0,
                                                               false)));
  task->start();
  timestamp = task->getStartTimestamp();
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
}

TEST(ResourceReservationRequests, consumableConsumption)
{
  ruler::ContinuousConsumableResourcePtr r1(
      new ruler::ContinuousConsumableResource("r1", "resource 1", 10.0, 4.0));
  ruler::DiscreteConsumableResourcePtr r2(
      new ruler::DiscreteConsumableResource("r2", "resource 2", 5l, 3l));
  ruler::UnaryConsumableResourcePtr r3(
      new ruler::UnaryConsumableResource("r3", "resource 3", true));
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  task->addResourceReservationRequest(
      ruler::ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              task, r1, utilities::functions::ContinuousStepFunctionPtr(
                            new utilities::functions::ContinuousStepFunction(
                                1.0, 3.0)))));
  task->addResourceReservationRequest(
      ruler::DiscreteConsumableResourceReservationRequestPtr(
          new ruler::DiscreteConsumableResourceReservationRequest(
              task, r2,
              utilities::functions::DiscreteStepFunctionPtr(
                  new utilities::functions::DiscreteStepFunction(2.0, 2.0)))));
  task->addResourceReservationRequest(
      ruler::UnaryConsumableResourceReservationRequestPtr(
          new ruler::UnaryConsumableResourceReservationRequest(task, r3, 6.0)));
  task->start();
  timestamp = task->getStartTimestamp();
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
}

TEST(ResourceReservationRequests, reusable)
{
  ruler::ContinuousReusableResourcePtr r1(
      new ruler::ContinuousReusableResource("r1", "resource 1", 10.0, 4.0));
  ruler::DiscreteReusableResourcePtr r2(
      new ruler::DiscreteReusableResource("r2", "resource 2", 5l, 3l));
  ruler::UnaryReusableResourcePtr r3(
      new ruler::UnaryReusableResource("r3", "resource 3", true));
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::PreemptiveTaskPtr task(
      new ruler::PreemptiveTask("t", "task", expected_start, expected_end));
  task->addResourceReservationRequest(
      ruler::ContinuousReusableResourceReservationRequestPtr(
          new ruler::ContinuousReusableResourceReservationRequest(task, r1, 3.0,
                                                                  1.0)));
  task->addResourceReservationRequest(
      ruler::DiscreteReusableResourceReservationRequestPtr(
          new ruler::DiscreteReusableResourceReservationRequest(task, r2, 2l,
                                                                2.0)));
  task->addResourceReservationRequest(
      ruler::UnaryReusableResourceReservationRequestPtr(
          new ruler::UnaryReusableResourceReservationRequest(task, r3, 6.0)));
  task->start();
  timestamp = task->getStartTimestamp();
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
}

TEST(ResourceSharing, task1and2)
{
  tolerance *= 50;
  double d(0.5);
  ruler::ContinuousConsumableResourcePtr r1(
      new ruler::ContinuousConsumableResource("r1", "resource 1", 5.0, 3.0));
  ruler::DiscreteReusableResourcePtr r2(
      new ruler::DiscreteReusableResource("r2", "resource 2", 6l, 5l));
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(0.5 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(4.5 * d), timestamp + ros::Duration(5.5 * d)));
  ruler::PreemptiveTaskPtr t1(
      new ruler::PreemptiveTask("t1", "task1", expected_start, expected_end));
  ruler::TaskPtr t2(
      new ruler::Task("t2", "task2", expected_start, expected_end));
  t1->addResourceReservationRequest(
      ruler::ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              t1, r1, utilities::functions::ContinuousLinearFunctionPtr(
                          new utilities::functions::ContinuousLinearFunction(
                              0.0 * d, 4.0 * d, 0.0, 4.0)))));
  t1->addResourceReservationRequest(
      ruler::DiscreteReusableResourceReservationRequestPtr(
          new ruler::DiscreteReusableResourceReservationRequest(t1, r2, 2l)));
  t2->addResourceReservationRequest(
      ruler::ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              t2, r1, utilities::functions::ContinuousLinearFunctionPtr(
                          new utilities::functions::ContinuousLinearFunction(
                              0.0 * d, 4.0 * d, 0.0, 4.0)),
              false)));
  t2->addResourceReservationRequest(
      ruler::DiscreteReusableResourceReservationRequestPtr(
          new ruler::DiscreteReusableResourceReservationRequest(t2, r2, 1l)));
  ros::Duration duration(d);
  ros::Rate rate(duration);
  rate.sleep(); // t = 0
  rate.sleep(); // t = 1
  t1->start();
  rate.sleep(); // t = 2
  t2->start();
  rate.sleep(); // t = 3
  t1->interrupt();
  rate.sleep(); // t = 4
  rate.sleep(); // t = 5
  t1->resume();
  rate.sleep(); // t = 6
  t2->finish();
  rate.sleep(); // t = 7
  t1->finish();
  // testing r1
  timestamp = t1->getStartTimestamp() - ros::Duration(1.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.0 - r1->getLevel(timestamp + ros::Duration(0.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.0 - r1->getLevel(timestamp + ros::Duration(0.5 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.0 - r1->getLevel(timestamp + ros::Duration(1.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.5 - r1->getLevel(timestamp + ros::Duration(1.5 * d))));
  timestamp = t2->getStartTimestamp() - ros::Duration(2.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 - 1.0 - r1->getLevel(timestamp + ros::Duration(2.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 1.0 - r1->getLevel(timestamp + ros::Duration(2.5 * d))));
  timestamp = t1->getLastInterruptionTimestamp() - ros::Duration(3.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 - 1.0 - r1->getLevel(timestamp + ros::Duration(3.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.5 - r1->getLevel(timestamp + ros::Duration(3.5 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 - 0.0 - r1->getLevel(timestamp + ros::Duration(4.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 + 0.5 - r1->getLevel(timestamp + ros::Duration(4.5 * d))));
  timestamp = t1->getLastResumeTimestamp() - ros::Duration(5.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 + 1.0 - r1->getLevel(timestamp + ros::Duration(5.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 + 1.0 - r1->getLevel(timestamp + ros::Duration(5.5 * d))));
  timestamp = t2->getEndTimestamp() - ros::Duration(6.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 + 1.0 - r1->getLevel(timestamp + ros::Duration(6.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 + 0.5 - r1->getLevel(timestamp + ros::Duration(6.5 * d))));
  timestamp = t1->getEndTimestamp() - ros::Duration(7.0 * d);
  EXPECT_GE(tolerance,
            fabs(3.0 + 0.0 - r1->getLevel(timestamp + ros::Duration(7.0 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 + 0.0 - r1->getLevel(timestamp + ros::Duration(7.5 * d))));
  EXPECT_GE(tolerance,
            fabs(3.0 + 0.0 - r1->getLevel(timestamp + ros::Duration(8.0 * d))));
  // testing r2
  timestamp = t1->getStartTimestamp() - ros::Duration(1.0 * d);
  EXPECT_EQ(5 - 0, r2->getLevel(timestamp + ros::Duration(0.0 * d)));
  EXPECT_EQ(5 - 0, r2->getLevel(timestamp + ros::Duration(0.5 * d)));
  EXPECT_EQ(5 - 0, r2->getLevel(timestamp + ros::Duration(1.0 * d)));
  EXPECT_EQ(5 - 2, r2->getLevel(timestamp + ros::Duration(1.5 * d)));
  timestamp = t2->getStartTimestamp() - ros::Duration(2.0 * d);
  EXPECT_EQ(5 - 2, r2->getLevel(timestamp + ros::Duration(2.0 * d)));
  EXPECT_EQ(5 - 3, r2->getLevel(timestamp + ros::Duration(2.5 * d)));
  timestamp = t1->getLastInterruptionTimestamp() - ros::Duration(3.0 * d);
  EXPECT_EQ(5 - 3, r2->getLevel(timestamp + ros::Duration(3.0 * d)));
  EXPECT_EQ(5 - 1, r2->getLevel(timestamp + ros::Duration(3.5 * d)));
  EXPECT_EQ(5 - 1, r2->getLevel(timestamp + ros::Duration(4.0 * d)));
  EXPECT_EQ(5 - 1, r2->getLevel(timestamp + ros::Duration(4.5 * d)));
  timestamp = t1->getLastResumeTimestamp() - ros::Duration(5.0 * d);
  EXPECT_EQ(5 - 1, r2->getLevel(timestamp + ros::Duration(5.0 * d)));
  EXPECT_EQ(5 - 3, r2->getLevel(timestamp + ros::Duration(5.5 * d)));
  timestamp = t2->getEndTimestamp() - ros::Duration(6.0 * d);
  EXPECT_EQ(5 - 3, r2->getLevel(timestamp + ros::Duration(6.0 * d)));
  EXPECT_EQ(5 - 2, r2->getLevel(timestamp + ros::Duration(6.5 * d)));
  timestamp = t1->getEndTimestamp() - ros::Duration(7.0 * d);
  EXPECT_EQ(5 - 2, r2->getLevel(timestamp + ros::Duration(7.0 * d)));
  EXPECT_EQ(5 - 0, r2->getLevel(timestamp + ros::Duration(7.5 * d)));
  EXPECT_EQ(5 - 0, r2->getLevel(timestamp + ros::Duration(8.0 * d)));
}

TEST(Task, distance)
{
  std::list<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose p;
  waypoints.push_back(p);
  p.position.x = 3.0;
  waypoints.push_back(p);
  p.position.x = 0.0;
  p.position.y = 4.0;
  waypoints.push_back(p);
  p.position.y = 0.0;
  waypoints.push_back(p);
  double d(1.0);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(5.0 * d), timestamp + ros::Duration(7.5 * d)));
  ruler::TaskPtr task(
      new ruler::Task("t", "task", expected_start, expected_end, waypoints));
  EXPECT_GE(tolerance, fabs(12.0 - task->getDistance()));
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
  continuous_step.reset(
      new utilities::functions::ContinuousStepFunction(d0, q0, qf));
  continuous_linear.reset(
      new utilities::functions::ContinuousLinearFunction(d0, df, q0, qf));
  continuous_exponential.reset(
      new utilities::functions::ContinuousExponentialFunction(d0, df, q0, qf));
  discrete_step.reset(
      new utilities::functions::DiscreteStepFunction(d0, q0, qf));
  discrete_linear.reset(
      new utilities::functions::DiscreteLinearFunction(d0, df, q0, qf));
  discrete_exponential.reset(
      new utilities::functions::DiscreteExponentialFunction(d0, df, q0, qf));
  unary_pulse.reset(new utilities::functions::UnaryPulseFunction(1.0, 1.5));
  unary_step.reset(new utilities::functions::UnaryStepFunction(0.25));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
