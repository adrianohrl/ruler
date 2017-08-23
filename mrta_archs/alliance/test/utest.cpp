/**
 *  This source file tests the main alliance namespace classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <gtest/gtest.h>

void init()
{
}

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "alliance_node");
  //ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
