/**
 *  This source file implements the main function that tests the ruler library.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/task_execution_simulator_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ruler_test::TaskExecutionSimulatorNode* node =
      new ruler_test::TaskExecutionSimulatorNode();
  node->spin();
  delete node;
  return EXIT_SUCCESS;
}
