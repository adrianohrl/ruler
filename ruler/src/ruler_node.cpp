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
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::RulerNodePtr node(new nodes::RulerNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
