#include "alliance/sensory_evaluator.h"

namespace alliance
{
SensoryEvaluator::SensoryEvaluator() {}

SensoryEvaluator::~SensoryEvaluator()
{
  sensory_feedback_pub_.shutdown();
}

void SensoryEvaluator::initialize(const ros::NodeHandlePtr &nh, const BehavedRobot &robot, const Task &task, const std::list<SensorPtr> &sensors)
{
  nh_ = nh;
  sensory_feedback_pub_ = nh_->advertise<alliance_msgs::SensoryFeedback>("/alliance/sensory_feedback", 10);
  sensory_feedback_msg_.header.frame_id = robot.getId();
  sensory_feedback_msg_.task_id = task.getId();
}

void SensoryEvaluator::process()
{
  sensory_feedback_msg_.header.stamp = ros::Time::now();
  sensory_feedback_msg_.applicable = isApplicable();
  sensory_feedback_pub_.publish(sensory_feedback_msg_);
}
}
