/**
 *  This source file implements the main function that tests the ruler library.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/task_generator_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_generator_node");
  ruler_test::TaskGeneratorNode* node = new ruler_test::TaskGeneratorNode();
  node->run();
  delete node;
  return EXIT_SUCCESS;
}
