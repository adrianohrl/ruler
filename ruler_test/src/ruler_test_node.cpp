/**
 *  This source file implements the main function that tests the ruler library.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <ros/ros.h>
#include <ruler/ruler.h>
#include <utilities/utilities.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ruler_test_node");
  ros::NodeHandle nh;
  ROS_INFO("Testing ruler library ...");
  ruler::ContinuousConsumableResource* r1 =
      new ruler::ContinuousConsumableResource("type1", "r1", 10.5, 1.2);
  ruler::ContinuousReusableResource* r2 =
      new ruler::ContinuousReusableResource("type2", "r2", 5.6, 2.4);
  ruler::DiscreteConsumableResource* r3 =
      new ruler::DiscreteConsumableResource("type3", "r3", 7, 3);
  ruler::DiscreteReusableResource* r4 =
      new ruler::DiscreteReusableResource("type4", "r4", 5, 4);
  ruler::UnaryConsumableResource* r5 =
      new ruler::UnaryConsumableResource("type5", "r5");
  ruler::UnaryReusableResource* r6 =
      new ruler::UnaryReusableResource("type6", "r6", false);
  ruler::Task* t1 = new ruler::Task("t1", "task 1", ros::Duration(10));
  ruler::Task* t2 = new ruler::Task("t2", "task 2", ros::Duration(20));
  ros::Duration d(1.0);
  utilities::ContinuousStepFunction* f1 =
      new utilities::ContinuousStepFunction(0.0, 5 * d.toSec(), 0.0, 10.0);
  utilities::DiscreteStepFunction* f2 =
      new utilities::DiscreteStepFunction(0.5, 2.5 * d.toSec(), 0.0, 4.0);
  t1->addResource(r1);
  t1->addResource(r2);
  t1->addResource(r3);
  t1->addResource(r4);
  t1->addResource(r5);
  t1->addResource(r6);
  try
  {
    t1->start();
    r1->produce(t1, f1);
    r2->require(t1, 1.5);
    r3->consume(t1, f2);
    r4->require(t1, 1);
    r5->consume(t1);
    r6->require(t1);
    d.sleep();
    ROS_INFO_STREAM("Duration after started: " << t1->getDuration() << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->interrupt();
    d.sleep();
    ROS_INFO_STREAM("Duration after interrupted 1: " << t1->getDuration()
                                                     << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->resume();
    d.sleep();
    ROS_INFO_STREAM("Duration after resumed 1: " << t1->getDuration()
                                                 << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->interrupt();
    d.sleep();
    ROS_INFO_STREAM("Duration after interrupted 2: " << t1->getDuration()
                                                     << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->resume();
    d.sleep();
    ROS_INFO_STREAM("Duration after resumed 2: " << t1->getDuration()
                                                 << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->interrupt();
    d.sleep();
    ROS_INFO_STREAM("Duration after interrupted 3: " << t1->getDuration()
                                                     << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    t1->finish();
    d.sleep();
    ROS_INFO_STREAM("Duration after finished: " << t1->getDuration()
                                                << " [s].");
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
    d.sleep();
    ROS_INFO_STREAM(r1 << " level: " << r1->getLevel());
    ROS_INFO_STREAM(r2 << " level: " << r2->getLevel());
    ROS_INFO_STREAM(r3 << " level: " << r3->getLevel());
    ROS_INFO_STREAM(r4 << " level: " << r4->getLevel());
    ROS_INFO_STREAM(r5 << " level: " << r5->getLevel());
    ROS_INFO_STREAM(r6 << " level: " << r6->getLevel());
  }
  catch (utilities::Exception e)
  {
  }
  delete t1;
  t1 = NULL;
  delete t2;
  t2 = NULL;
  delete r1;
  r1 = NULL;
  delete r2;
  r2 = NULL;
  delete r3;
  r3 = NULL;
  delete r4;
  r4 = NULL;
  delete r5;
  r5 = NULL;
  delete r6;
  r6 = NULL;
  return 0;
}
