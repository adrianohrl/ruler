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
  RulerNode(ros::NodeHandlePtr nh, float loop_rate = 30.0);
  virtual ~RulerNode();

private:
  ruler::Robot* robot_;
  ros::Publisher resources_pub_;
  std::map<std::string, ros::Publisher> resource_pubs_;
  std::list<MetricsServiceServer*> metrics_srvs_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  bool calculateMetrics();
};

typedef boost::shared_ptr<RulerNode> RulerNodePtr;
}

#endif // _RULER_NODE_H_
