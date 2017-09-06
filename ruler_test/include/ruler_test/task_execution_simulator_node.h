/**
 *  This header file defines the TaskExecutionSimulatorNode class, which is
 *based on the ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TEST_TASK_EXECUTION_SIMULATOR_NODE_H_
#define _RULER_TEST_TASK_EXECUTION_SIMULATOR_NODE_H_

#include <ruler/ruler.h>
#include <utilities/ros_node.h>

namespace ruler_test
{
class TaskExecutionSimulatorNode : public utilities::ROSNode
{
public:
  TaskExecutionSimulatorNode(ros::NodeHandlePtr nh,
                             float loop_rate = 30.0);
  virtual ~TaskExecutionSimulatorNode();

private:
  virtual void readParameters();
  virtual void controlLoop();
};

typedef boost::scoped_ptr<TaskExecutionSimulatorNode> TaskExecutionSimulatorNodePtr;
}

#endif // _RULER_TEST_TASK_EXECUTION_SIMULATOR_NODE_H_
