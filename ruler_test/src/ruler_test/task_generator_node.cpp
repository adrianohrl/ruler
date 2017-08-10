/**
 *  This source file implements the TaskGeneratorNode class, which is based on
 *the
 *ROSNode helper class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler_test/task_generator_node.h"

namespace ruler_test
{
TaskGeneratorNode::TaskGeneratorNode(ros::NodeHandle* nh, float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate)
{
  task_announcements_pub_ = nh->advertise<ruler_msgs::Task>("tasks", 10);
}

TaskGeneratorNode::~TaskGeneratorNode() { task_announcements_pub_.shutdown(); }

void TaskGeneratorNode::readParameters()
{
  ros::Time timestamp(ros::Time::now());
  ros::NodeHandle pnh("~/task_announcements/");
  int tasks_size;
  pnh.param("size", tasks_size, 0);
  std::string task_id;
  double min_delay, max_delay;
  for (int i(0); i < tasks_size; i++)
  {
    std::stringstream ss;
    ss << "announcement" << i << "/";
    pnh.param(ss.str() + "task_id", task_id, std::string(""));
    if (task_id.empty())
    {
      ROS_ERROR_STREAM("Invalid task id: '" << task_id << "'.");
      continue;
    }
    pnh.param(ss.str() + "min_delay", min_delay, 0.0);
    if (min_delay < 0.0)
    {
      ROS_ERROR_STREAM(
          "The minimum task announcement delay must not be negative.");
      continue;
    }
    pnh.param(ss.str() + "max_delay", max_delay, 0.0);
    if (min_delay > max_delay)
    {
      ROS_ERROR_STREAM("The maximum task announcement delay must not be less "
                       "than the minimum one.");
      continue;
    }
    TaskAnnouncement announcement(task_id, min_delay, max_delay);
    if (contains(announcement))
    {
      ROS_WARN_STREAM("Ignored already imported task announcement: "
                      << announcement.task_id_);
      continue;
    }
    task_announcements_.push_back(announcement);
    ROS_INFO_STREAM("Added task announcement: " << announcement.task_id_);
  }
  if (task_announcements_.empty())
  {
    ROS_FATAL("None task announcement was imported.");
    ROSNode::shutdown();
  }
}

void TaskGeneratorNode::controlLoop()
{
  ros::NodeHandle* nh = ROSNode::getNodeHandle();
  std::vector<TaskAnnouncement>::iterator it(task_announcements_.begin());
  while (it != task_announcements_.end())
  {
    TaskAnnouncement* announcement = &*it;
    if (ros::Time::now() >= announcement->delay_timestamp_)
    {
      ROS_INFO_STREAM("Announcing " << announcement->task_id_ << " ...");
      task_announcements_pub_.publish(announcement->toMsg());
      it = task_announcements_.erase(it);
      continue;
    }
    it++;
  }
}

bool TaskGeneratorNode::contains(const TaskAnnouncement& announcement) const
{
  for (int i(0); i < task_announcements_.size(); i++)
  {
    if (task_announcements_[i] == announcement)
    {
      return true;
    }
  }
  return false;
}
}
