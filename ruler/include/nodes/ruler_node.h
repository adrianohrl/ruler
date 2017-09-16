/**
 *  This header file defines the RulerNode class, which is based on the ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_NODE_H_
#define _RULER_NODE_H_

#include <map>
#include "nodes/metrics_service_server.h"
#include "ruler/ruler.h"
#include "utilities/ros_node.h"

namespace nodes
{
class RulerNode : public utilities::ROSNode
{
public:
  RulerNode(const ros::NodeHandlePtr& nh, const ros::Rate& loop_rate = ros::Rate(30.0));
  virtual ~RulerNode();

private:
  typedef std::pair<std::string, ros::Publisher> pair;
  typedef std::map<std::string, ros::Publisher>::iterator pub_iterator;
  typedef std::map<std::string, ros::Publisher>::const_iterator pub_const_iterator;
  typedef std::list<MetricsServiceServerPtr>::iterator srv_iterator;
  typedef std::list<MetricsServiceServerPtr>::const_iterator srv_const_iterator;
  ruler::RobotPtr robot_;
  ros::Publisher resources_pub_;
  std::map<std::string, ros::Publisher> resource_pubs_;
  std::list<MetricsServiceServerPtr> metrics_srvs_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  bool calculateMetrics();
};

typedef boost::shared_ptr<RulerNode> RulerNodePtr;
}

#endif // _RULER_NODE_H_
