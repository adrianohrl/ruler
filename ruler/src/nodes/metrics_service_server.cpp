/**
 *  This header file defines the MetricsServiceServer class, which is based on
 *the ROSServiceServer abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/metrics_service_server.h"

namespace nodes
{

MetricsServiceServer::MetricsServiceServer(const ros::NodeHandlePtr& nh,
                                           const std::string& name)
    : ROSServiceServer<ruler_msgs::CalculateMetrics::Request,
                       ruler_msgs::CalculateMetrics::Response>::
          ROSServiceServer(nh->advertiseService(
              name, &MetricsServiceServer::callback, this)),
      loader_("ruler", "ruler::MetricsEstimator")
{
}

MetricsServiceServer::~MetricsServiceServer() {}

void MetricsServiceServer::readPlugins(const ruler::RobotPtr& robot,
                                       const std::string& ns)
{
  ros::NodeHandle pnh(ns + "/plugins");
  int size;
  pnh.param("size", size, 0);
  std::string name;
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "plugin" << i << "/";
    pnh.param(ss.str() + "name", name, std::string(""));
    if (name.empty())
    {
      ROS_ERROR("The plugin name must not be empty.");
      continue;
    }
    try
    {
      ruler::MetricsEstimatorPtr estimator(
          loader_.createInstance(name.c_str()));
      estimator->initialize(robot);
      estimators_.push_back(estimator);
      ROS_INFO_STREAM("   Added the " << name << " plugin to the " << str()
                                      << " service server.");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s",
                ex.what());
    }
  }
  if (estimators_.empty())
  {
    throw utilities::Exception(
        "None metrics estimator plugin was added to the " + str() +
        " service server.");
  }
}

void MetricsServiceServer::shutdown()
{
  for (iterator it(estimators_.begin()); it != estimators_.end(); it++)
  {
    ruler::MetricsEstimatorPtr estimator(*it);
    estimator->shutdown();
  }
}

bool MetricsServiceServer::callback(
    ruler_msgs::CalculateMetrics::Request& request,
    ruler_msgs::CalculateMetrics::Response& response)
{
  for (iterator it(estimators_.begin()); it != estimators_.end(); it++)
  {
    ruler::MetricsEstimatorPtr estimator(*it);
    response.metrics += estimator->calculate(request.task);
  }
  return true;
}
}
