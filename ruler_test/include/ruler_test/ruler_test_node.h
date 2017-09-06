/**
 *  This header file defines the RulerTestNode class, which is based on the ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TEST_NODE_H_
#define _RULER_TEST_NODE_H_

#include <ruler/ruler.h>
#include <utilities/ros_node.h>

namespace ruler_test
{
class RulerTestNode : public utilities::ROSNode
{
public:
  RulerTestNode(ros::NodeHandlePtr nh, float loop_rate = 30.0);
  virtual ~RulerTestNode();

private:
  ros::Publisher resources_pub_;
  std::vector<ros::Publisher> resource_pubs_;
  std::vector<ruler::ResourceInterface*> resources_;
  virtual void readParameters();
  virtual void init();
  virtual void controlLoop();
  bool contains(const ruler::ResourceInterface& resource) const;
};

typedef boost::scoped_ptr<RulerTestNode> RulerTestNodePtr;
}

#endif // _RULER_TEST_NODE_H_
