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

double tolerance = 1e-4;
std::vector<double> d;
utilities::Function* step;
utilities::Function* linear;
utilities::Function* exponential;
std::map<double, double> q_step_asc;
std::map<double, double> q_step_des;
std::map<double, double> q_linear_asc;
std::map<double, double> q_linear_des;
std::map<double, double> q_exponential_asc;
std::map<double, double> q_exponential_des;

TEST(Functions, step)
{
  step->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_step_asc[d[i]] - step->getValue(d[i])));
  }
  step->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_step_des[d[i]] - step->getValue(d[i])));
  }
}

TEST(Functions, linear)
{
  linear->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_linear_asc[d[i]] - linear->getValue(d[i])));
  }
  linear->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_linear_des[d[i]] - linear->getValue(d[i])));
  }
}

TEST(Functions, exponential)
{
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_exponential_asc[d[i]] - exponential->getValue(d[i])));
  }
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance,
              fabs(q_exponential_des[d[i]] - exponential->getValue(d[i])));
  }
}

TEST(Task, start)
{
  ruler::Task* task = new ruler::Task("t", "task");
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
  delete resource;
  resource = NULL;
}

TEST(Task, interrupt)
{
  ruler::Task* task = new ruler::Task("t", "task");
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
  ruler::Task* task = new ruler::Task("t", "task");
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
  ruler::Task* task = new ruler::Task("t", "task");
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
  ruler::Task* task = new ruler::Task("t", "task");
  ruler::UnaryConsumableResource* resource =
      new ruler::UnaryConsumableResource("r", "resource");
  task->addResource(resource);
  task->start();
  ros::Time t(task->getStartTimestamp());
  double c(1000), l0(300);
  profile = new ruler::Profile<utilities::ContinuousSignalType>(c, l0);
  profile->addTaskFunction(new ruler::TaskFunction(task, step));
  profile->addTaskFunction(new ruler::TaskFunction(task, linear));
  profile->addTaskFunction(new ruler::TaskFunction(task, exponential));
  step->setAscending(true);
  linear->setAscending(true);
  exponential->setAscending(true);
  utilities::ContinuousSignalType el(0.0), l(0.0);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(true);
  linear->setAscending(true);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(true);
  linear->setAscending(false);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(true);
  linear->setAscending(false);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(false);
  linear->setAscending(true);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(false);
  linear->setAscending(true);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(false);
  linear->setAscending(false);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  step->setAscending(false);
  linear->setAscending(false);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    el = l0 + q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]];
    l = profile->getLevel(t + ros::Duration(d[i]));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_LE(l, c);
  }
  delete profile;
  profile = NULL;
}

/*TEST(Resources, reusable)
{

}*/

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
  q_step_asc.insert(std::pair<double, double>(d[2], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[3], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[4], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[5], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[6], 8.1));
  q_step_asc.insert(std::pair<double, double>(d[7], 8.1));
  // descending step function expected quantities
  q_step_des.insert(std::pair<double, double>(d[0], 8.1));
  q_step_des.insert(std::pair<double, double>(d[1], 8.1));
  q_step_des.insert(std::pair<double, double>(d[2], 6.1));
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
  step = new utilities::StepFunction(d0, df, q0, qf);
  linear = new utilities::LinearFunction(d0, df, q0, qf);
  exponential = new utilities::ExponentialFunction(d0, df, q0, qf);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
