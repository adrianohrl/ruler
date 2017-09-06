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
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::NodePtr node(new nodes::Node(nh));
  node->run();
  return EXIT_SUCCESS;
}
