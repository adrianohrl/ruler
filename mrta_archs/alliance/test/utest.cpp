/**
 *  This source file tests the main alliance namespace classes.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "alliance/alliance.h"
#include "utilities/utilities.h"
#include "utilities/beacon_signal_subject.h"
#include <gtest/gtest.h>

using namespace alliance;

TaskPtr t1;
RobotPtr r1, r2, r3;
InterCommunicationPtr r1_monitor, r2_monitor, r3_monitor;
utilities::BeaconSignalSubjectPtr r1_subject, r2_subject, r3_subject;

void notify(const RobotPtr& robot, const TaskPtr& task, const ros::Time& timestamp)
{
  alliance_msgs::BeaconSignal msg;
  msg.header.frame_id = robot->getId();
  msg.task_id = task->getId();
  msg.header.stamp = timestamp;
  utilities::BeaconSignalEventPtr event;
  event.reset(new utilities::BeaconSignalEvent(r1_subject, msg));
  r1_subject->notify(event);
  event.reset(new utilities::BeaconSignalEvent(r2_subject, msg));
  r2_subject->notify(event);
  event.reset(new utilities::BeaconSignalEvent(r3_subject, msg));
  r3_subject->notify(event);
}

TEST(MotivationalBehaviour, timeout)
{
  ros::Time timestamp(ros::Time::now());
  notify(r1, t1, timestamp + ros::Duration(0.5));
  notify(r2, t1, timestamp + ros::Duration(0.5));
  notify(r3, t1, timestamp + ros::Duration(0.5));
  notify(r2, t1, timestamp + ros::Duration(1.0));
  notify(r3, t1, timestamp + ros::Duration(1.0));
  notify(r3, t1, timestamp + ros::Duration(1.5));
  notify(r3, t1, timestamp + ros::Duration(2.0));
  notify(r3, t1, timestamp + ros::Duration(2.5));
  notify(r3, t1, timestamp + ros::Duration(3.0));
  notify(r3, t1, timestamp + ros::Duration(3.5));
  EXPECT_TRUE(r1_monitor->received(r2->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r1_monitor->received(r3->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r2_monitor->received(r1->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r2_monitor->received(r3->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r3_monitor->received(r1->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_TRUE(r3_monitor->received(r2->getId(), timestamp, timestamp + ros::Duration(3.5)));
  EXPECT_FALSE(r1_monitor->received(r2->getId(), timestamp + ros::Duration(2.5), timestamp + ros::Duration(3.5))); // not ok
  EXPECT_FALSE(r2_monitor->received(r1->getId(), timestamp + ros::Duration(2.0), timestamp + ros::Duration(3.5))); // not ok
  EXPECT_FALSE(r3_monitor->received(r1->getId(), timestamp + ros::Duration(2.0), timestamp + ros::Duration(3.5))); // not ok
  EXPECT_FALSE(r3_monitor->received(r2->getId(), timestamp + ros::Duration(2.5), timestamp + ros::Duration(3.5))); // not ok
}

void init()
{
  t1.reset(new Task("t1", "task1"));
  r1_subject.reset(new utilities::BeaconSignalSubject("r1/t1/subject"));
  r2_subject.reset(new utilities::BeaconSignalSubject("r2/t1/subject"));
  r3_subject.reset(new utilities::BeaconSignalSubject("r3/t1/subject"));

  r1.reset(new Robot("r1", "robot 1"));
  r1->setBroadcastRate(ros::Rate(1.0));
  r1->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r1bs1(new BehaviourSet(r1, t1, ros::Duration(10.0)));
  r1bs1->init();
  r1bs1->setActivationThreshold(60);
  r1bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r1bs1->setImpatience(2.5);
  r1_monitor = r1bs1->getMotivationalBehaviour()->getInterCommunication();
  r1_subject->registerObserver(r1_monitor);
  r1->addBehaviourSet(r1bs1);

  r2.reset(new Robot("r2", "robot 2"));
  r2->setBroadcastRate(ros::Rate(1.0));
  r2->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r2bs1(new BehaviourSet(r2, t1, ros::Duration(10.0)));
  r2bs1->init();
  r2bs1->setActivationThreshold(60);
  r2bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r2bs1->setImpatience(2.5);
  r2_monitor = r2bs1->getMotivationalBehaviour()->getInterCommunication();
  r2_subject->registerObserver(r2_monitor);
  r1->addBehaviourSet(r2bs1);

  r3.reset(new Robot("r3", "robot 3"));
  r3->setBroadcastRate(ros::Rate(1.0));
  r3->setTimeoutDuration(ros::Duration(1.5));
  BehaviourSetPtr r3bs1(new BehaviourSet(r3, t1, ros::Duration(10.0)));
  r3bs1->init();
  r3bs1->setActivationThreshold(60);
  r3bs1->setAcquiescence(ros::Duration(0.75), ros::Duration(7.5));
  r3bs1->setImpatience(2.5);
  r3_monitor = r3bs1->getMotivationalBehaviour()->getInterCommunication();
  r3_subject->registerObserver(r3_monitor);
  r1->addBehaviourSet(r3bs1);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "alliance_test_node");
  ros::NodeHandle nh;
  init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
