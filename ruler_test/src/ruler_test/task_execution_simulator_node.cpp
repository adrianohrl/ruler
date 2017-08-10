/**
 *  This source file implements the TaskExecutionSimulatorNode class, which is
 *based on the ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/task_execution_simulator_node.h"

namespace ruler_test
{
TaskExecutionSimulatorNode::TaskExecutionSimulatorNode(ros::NodeHandle* nh,
                                                       float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate)
{
}

TaskExecutionSimulatorNode::~TaskExecutionSimulatorNode() {}

void TaskExecutionSimulatorNode::readParameters() {}

void TaskExecutionSimulatorNode::controlLoop() {}
}
