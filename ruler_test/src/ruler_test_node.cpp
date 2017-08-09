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
  ruler_test::RulerTestNode* node = new ruler_test::RulerTestNode();
  node->spin();
  delete node;
  return EXIT_SUCCESS;
}
