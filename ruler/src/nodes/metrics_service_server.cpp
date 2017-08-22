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

MetricsServiceServer::MetricsServiceServer(ros::NodeHandle* nh,
                                           std::string name)
    : ROSServiceServer<ruler_msgs::CalculateMetrics::Request,
                       ruler_msgs::CalculateMetrics::Response>::
          ROSServiceServer(nh->advertiseService(
              name, &MetricsServiceServer::callback, this)),
      loader_("ruler", "ruler::MetricsEstimator")
{
}

MetricsServiceServer::~MetricsServiceServer() {}

void MetricsServiceServer::readPlugins(ruler::Robot* robot, std::string ns)
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
      boost::shared_ptr<ruler::MetricsEstimator> estimator(
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

bool MetricsServiceServer::callback(
    ruler_msgs::CalculateMetrics::Request& request,
    ruler_msgs::CalculateMetrics::Response& response)
{
  ruler_msgs::CalculateMetrics srv;
  std::list<boost::shared_ptr<ruler::MetricsEstimator> >::const_iterator it(
      estimators_.begin());
  srv.response.metrics = 0.0;
  while (it != estimators_.end())
  {
    boost::shared_ptr<ruler::MetricsEstimator> estimator = *it;
    srv.response.metrics += estimator->calculate(request.task);
    it++;
  }
  return true;
}
}
