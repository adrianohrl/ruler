/**
 *  This header file defines the MetricsServiceServer class, which is based on
 *the ROSServiceServer abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_NODE_METRICS_SERVICE_SERVER_H_
#define _RULER_NODE_METRICS_SERVICE_SERVER_H_

#include <list>
#include <pluginlib/class_loader.h>
#include "ruler/metrics_estimator.h"
#include <ruler_msgs/CalculateMetrics.h>
#include "utilities/ros_service_server.h"

namespace nodes
{
class MetricsServiceServer
    : public utilities::ROSServiceServer<ruler_msgs::CalculateMetrics::Request,
                                         ruler_msgs::CalculateMetrics::Response>
{
public:
  MetricsServiceServer(const ros::NodeHandlePtr& nh, const std::string& name);
  virtual ~MetricsServiceServer();
  void readPlugins(const ruler::RobotPtr& robot, const std::string& ns);
  void shutdown();

private:
  typedef std::list<ruler::MetricsEstimatorPtr>::iterator iterator;
  typedef std::list<ruler::MetricsEstimatorPtr>::const_iterator const_iterator;
  pluginlib::ClassLoader<ruler::MetricsEstimator> loader_;
  std::list<ruler::MetricsEstimatorPtr> estimators_;
  virtual bool callback(ruler_msgs::CalculateMetrics::Request& request,
                        ruler_msgs::CalculateMetrics::Response& response);
};

typedef boost::shared_ptr<MetricsServiceServer> MetricsServiceServerPtr;
typedef boost::shared_ptr<MetricsServiceServer const>
    MetricsServiceServerConstPtr;
}

#endif // _RULER_NODE_METRICS_SERVICE_SERVER_H_
