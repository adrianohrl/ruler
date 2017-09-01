/**
 *  This source file implements the main function that runs the Node class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "node");
  nodes::Node* node = new nodes::Node();
  node->run();
  delete node;
  return EXIT_SUCCESS;
}
