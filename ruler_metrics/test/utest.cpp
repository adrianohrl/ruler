/**
 *  This source file tests and loads the ruler metrics plugins.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ruler/ruler.h>
#include "ruler/metrics_plugins.h"

double tolerance = 1e-5;
ruler::Robot* robot;
ruler::Task* task;

TEST(Plugins, calculus)
{
  ruler::MetricsEstimator* estimator;
  estimator = new ruler::BatteryConsumptionEstimator();
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(1.0 - estimator->calculate(*task)));
  delete estimator;
  estimator = new ruler::DisplacementEstimator();
  estimator->initialize(robot);
  EXPECT_GE(tolerance, fabs(-3.0 - estimator->calculate(*task)));
  delete estimator;
  estimator = NULL;
}

TEST(Plugins, loading)
{
  std::string plugin_name;
  std::string base_class_pkg("ruler");
  std::string base_class_type("ruler::MetricsEstimator");
  pluginlib::ClassLoader<ruler::MetricsEstimator> metrics_loader(
      base_class_pkg, base_class_type);
  boost::shared_ptr<ruler::MetricsEstimator> estimator;
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
  EXPECT_GE(tolerance, fabs(-3.0 - estimator->calculate(*task)));
  delete robot;
  robot = NULL;
  delete task;
  task = NULL;
}

void init()
{
  robot = new ruler::Robot();
  task = new ruler::Task("t", "task", ros::Duration(10));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_metrics_estimators_loader");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
