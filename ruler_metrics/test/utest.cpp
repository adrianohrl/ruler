/**
 *  This source file tests and loads the ruler metrics plugins.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/metrics_plugins.h"
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ruler/ruler.h>

double tolerance = 1e-5;
ruler::RobotPtr robot;
ruler::TaskPtr task;

TEST(Plugins, calculus)
{
  ruler::MetricsEstimatorPtr estimator;
  estimator.reset(new ruler::BatteryConsumptionEstimator());
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(1.0 - estimator->calculate(*task)));
  estimator.reset(new ruler::DisplacementEstimator());
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(12.0 - estimator->calculate(*task)));
}

TEST(Plugins, loading)
{
  std::string plugin_name;
  std::string base_class_pkg("ruler");
  std::string base_class_type("ruler::MetricsEstimator");
  pluginlib::ClassLoader<ruler::MetricsEstimator> metrics_loader(
      base_class_pkg, base_class_type);
  ruler::MetricsEstimatorPtr estimator;
  try
  {
    plugin_name = "ruler_metrics/battery_consumption_estimator";
    estimator = metrics_loader.createInstance(plugin_name.c_str());
    ROS_INFO("Metric: %.2f (using %s plugin)", estimator->calculate(*task),
             plugin_name.c_str());
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& ex)
  {
    SUCCEED();
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    FAIL() << "The plugin failed to load for some reason. Error: " << ex.what();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(1.0 - estimator->calculate(*task)));
  try
  {
    plugin_name = "ruler_metrics/displacement_estimator";
    estimator = metrics_loader.createInstance(plugin_name.c_str());
    estimator->calculate(*task);
    ADD_FAILURE() << "Didn't throw exception as expected.";
  }
  catch (const utilities::Exception& ex)
  {
    SUCCEED();
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    FAIL() << "The plugin failed to load for some reason. Error: " << ex.what();
  }
  catch (...)
  {
    FAIL() << "Uncaught exception.";
  }
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(12.0 - estimator->calculate(*task)));
}

void init()
{
  robot.reset(new ruler::Robot("robot1", "robot1"));
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
  double d(0.5);
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(new utilities::NoisyTime(
      timestamp + ros::Duration(0.5 * d), timestamp + ros::Duration(1.5 * d)));
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(4.5 * d), timestamp + ros::Duration(5.5 * d)));
  task.reset(
      new ruler::Task("t1", "task1", expected_start, expected_end, waypoints));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_metrics_estimators_loader");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
