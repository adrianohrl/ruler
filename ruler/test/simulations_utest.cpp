/**
 *  This source file tests the ruler simulation classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/ruler.h"
#include "utilities/utilities.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(Simulation, taskSimulation)
{
  double d(1.0);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp + ros::Duration(9.9 * d),
                               timestamp + ros::Duration(10.01 * d)));
  utilities::NoisyTimePtr expected_end(
      new utilities::NoisyTime(timestamp + ros::Duration(54.0 * d),
                               timestamp + ros::Duration(56.0 * d)));
  ruler::TaskPtr task(
      new ruler::Task("t", "task", expected_start, expected_end));
  ruler::UnaryConsumableResourcePtr resource(
      new ruler::UnaryConsumableResource("r", "resource"));
  task->addResource(resource);
  utilities::ContinuousNoisySignalPtr expected_sample_time(
      new utilities::ContinuousNoisySignal(0.5 * d, 0.005 * d));
  ruler::TaskSimulationPtr simulation(
      new ruler::TaskSimulation(task, expected_sample_time));
  timestamp = simulation->getSimulationStartTimestamp();
  ROS_WARN("%s", simulation->c_str());
  while (!task->hasFinished())
  {
    timestamp += ros::Duration(expected_sample_time->random());
    simulation->update(timestamp);
    ROS_WARN("%s", simulation->c_str());
  }
}

TEST(Simulation, batterySimulation)
{
  utilities::ContinuousNoisySignalPtr expected_sample_time(
      new utilities::ContinuousNoisySignal(0.05, 0.01));
  ruler::BatterySimulationPtr battery(
      new ruler::BatterySimulation("r1", expected_sample_time, 0.1));
  battery->init();
  ros::Time timestamp(battery->getSimulationStartTimestamp());
  utilities::NoisyDurationPtr expected_duration(
      new utilities::NoisyDuration(ros::Duration(25), 8));
  battery->discharge("t1", expected_duration, timestamp + ros::Duration(100));
  while (!battery->isEmpty(timestamp))
  {
    timestamp += ros::Duration(expected_sample_time->random());
    battery->update(timestamp);
  }
}

void init() {}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_simulations_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
