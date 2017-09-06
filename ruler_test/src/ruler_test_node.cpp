/**
 *  This source file implements the main function that tests the ruler library.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/ruler_test_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  ruler_test::RulerTestNodePtr node(new ruler_test::RulerTestNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
