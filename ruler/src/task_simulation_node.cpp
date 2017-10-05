/**
 *  This source file implements the main function that runs the
 *TaskSimulationNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/task_simulation_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::TaskSimulationNodePtr node(new nodes::TaskSimulationNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
