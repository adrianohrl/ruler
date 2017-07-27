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
    EXPECT_GE(tolerance, fabs(q_exponential_asc[d[i]] - exponential->getValue(d[i])));
  }
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    EXPECT_GE(tolerance, fabs(q_exponential_des[d[i]] - exponential->getValue(d[i])));
  }
}

TEST(Profiles, continuous)
{
  ruler::Profile<utilities::ContinuousSignalType>* p1;
  try
  {
    p1 = new ruler::Profile<utilities::ContinuousSignalType>(10.3, 12.1);
    delete p1;
    p1 = NULL;
    ADD_FAILURE() << "Didn't throw exception as expected";
  }
  catch (utilities::Exception e)
  {
    SUCCEED();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  double c(1000.0), l0(300.0);
  p1 = new ruler::Profile<utilities::ContinuousSignalType>(c, l0);
  p1->addTaskFunction(new ruler::TaskFunction(NULL, step));
  p1->addTaskFunction(new ruler::TaskFunction(NULL, linear));
  p1->addTaskFunction(new ruler::TaskFunction(NULL, exponential));
  step->setAscending(true);
  linear->setAscending(true);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(true);
  linear->setAscending(true);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_asc[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(true);
  linear->setAscending(false);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(true);
  linear->setAscending(false);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_asc[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(false);
  linear->setAscending(true);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_asc[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(false);
  linear->setAscending(true);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_des[d[i]] + q_linear_asc[d[i]] + q_exponential_des[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(false);
  linear->setAscending(false);
  exponential->setAscending(true);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_asc[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  step->setAscending(false);
  linear->setAscending(false);
  exponential->setAscending(false);
  for (int i(0); i < d.size(); i++)
  {
    double el(l0 + q_step_des[d[i]] + q_linear_des[d[i]] + q_exponential_des[d[i]]);
    double l(p1->getLevel(d[i], l0));
    EXPECT_GE(tolerance, fabs(el - l));
    EXPECT_GE(l, 0.0);
    EXPECT_GE(c, l);
  }
  delete p1;
  p1 = NULL;
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

int main(int argc, char **argv){
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
