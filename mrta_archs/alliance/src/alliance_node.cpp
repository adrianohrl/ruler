/**
 *  This source file implements the main function that runs the AllianceNode class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "nodes/alliance_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "alliance_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  nodes::AllianceNodePtr node(new nodes::AllianceNode(nh));
  node->run();
  return EXIT_SUCCESS;
}
