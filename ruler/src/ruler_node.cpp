/**
 *  This source file implements the main function that runs the RulerNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/ruler_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_node");
  nodes::RulerNode* node = new nodes::RulerNode();
  node->run();
  delete node;
  return EXIT_SUCCESS;
}
