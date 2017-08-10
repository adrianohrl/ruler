/**
 *  This header file defines the TaskGeneratorNode class, which is based on the
 *ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _TASK_GENERATOR_NODE_H_
#define _TASK_GENERATOR_NODE_H_

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <ruler/ruler.h>
#include <utilities/ros_node.h>

namespace ruler_test
{
struct TaskAnnouncement
{
  std::string task_id_;
  ros::Time delay_timestamp_;

  TaskAnnouncement(std::string task_id, double min_delay, double max_delay)
      : task_id_(task_id)
  {
    if (min_delay > max_delay)
    {
      throw utilities::Exception(
          "The minimum announcement timstamp must be before the maximum one.");
    }
    srand(time(NULL));
    double delay((rand() % 100 + 1) / 100.0 * (max_delay - min_delay) + min_delay);
    delay_timestamp_ = ros::Time::now() + ros::Duration(delay);
  }

  bool operator==(const TaskAnnouncement& announcement) const
  {
    return task_id_ == announcement.task_id_;
  }

  bool operator!=(const TaskAnnouncement& announcement) const
  {
    return task_id_ != announcement.task_id_;
  }

  ruler_msgs::Task toMsg() const
  {
    ruler_msgs::Task msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = task_id_;
    return msg;
  }
};

class TaskGeneratorNode : public utilities::ROSNode
{
public:
  TaskGeneratorNode(ros::NodeHandle* nh = new ros::NodeHandle(),
                    float loop_rate = 30.0);
  virtual ~TaskGeneratorNode();

private:
  ros::Publisher task_announcements_pub_;
  std::vector<TaskAnnouncement> task_announcements_;
  virtual void readParameters();
  virtual void controlLoop();
  bool contains(const TaskAnnouncement& announcement) const;
};
}

#endif // _TASK_GENERATOR_NODE_H_
