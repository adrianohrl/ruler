#include "nodes/task_simulation_node.h"

namespace nodes
{
TaskSimulationNode::TaskSimulationNode(const ros::NodeHandlePtr& nh,
                                       const ros::Rate& loop_rate)
    : ROSNode::ROSNode(nh, loop_rate), start_timestmap_(ros::Time::now())
{
}

TaskSimulationNode::~TaskSimulationNode() {}

void TaskSimulationNode::readParameters()
{
  ROSNode::readParameters();
  ros::NodeHandle pnh("~");
  double sample_time, sample_time_std;
  pnh.param("expected_sample_time/mean", sample_time, 0.0);
  pnh.param("expected_sample_time/standard_deviation", sample_time_std, 0.0);
  utilities::ContinuousNoisySignalPtr expected_sample_time(
      new utilities::ContinuousNoisySignal(sample_time, sample_time_std));
  pnh = ros::NodeHandle("~/tasks");
  int size;
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "task" << i << "/";
    std::string id, name;
    pnh.param(ss.str() + "id", id, std::string(""));
    pnh.param(ss.str() + "name", name, std::string(""));
    bool preemptive;
    pnh.param("preemptive", preemptive, false);
    double min_delay, max_delay;
    pnh.param("expected_start/min_delay", min_delay, 0.0);
    if (min_delay < 0.0)
    {
      ROS_WARN("The expected start minimum delay must not be negative.");
      min_delay = 0.0;
    }
    pnh.param("expected_start/max_delay", max_delay, min_delay);
    if (max_delay < min_delay)
    {
      ROS_WARN("The expected start maximum delay must not be lesser than its "
               "minimum one.");
      max_delay = min_delay;
    }
    utilities::NoisyTimePtr expected_start(
        new utilities::NoisyTime(start_timestmap_ + ros::Duration(min_delay),
                                 start_timestmap_ + ros::Duration(max_delay)));
    pnh.param("expected_end/min_delay", min_delay, max_delay);
    if (min_delay < max_delay)
    {
      ROS_WARN("The expected end minimum delay must not be lesser than the "
               "expected start maximum one.");
      min_delay = max_delay;
    }
    pnh.param("expected_end/max_delay", max_delay, min_delay);
    if (max_delay < min_delay)
    {
      ROS_WARN("The expected end maximum delay must not be lesser than its "
               "minimum one.");
      max_delay = min_delay;
    }
    utilities::NoisyTimePtr expected_end(
        new utilities::NoisyTime(start_timestmap_ + ros::Duration(min_delay),
                                 start_timestmap_ + ros::Duration(max_delay)));
    ss << "waypoints/";
    int waypoints_size;
    pnh.param(ss.str() + "size", waypoints_size, 0);
    std::list<geometry_msgs::Pose> waypoints;
    for (int j(0); j < waypoints_size; j++)
    {
      ss << "waypoint" << i << "/";
      geometry_msgs::Pose waypoint;
      pnh.param(ss.str() + "x", waypoint.position.x, 0.0);
      pnh.param(ss.str() + "y", waypoint.position.y, 0.0);
      waypoints.push_back(waypoint);
    }
    if (!preemptive)
    {
      ruler::TaskPtr task(
          new ruler::Task(id, name, expected_start, expected_end, waypoints));
      ruler::TaskSimulationPtr simulation(
          new ruler::TaskSimulation(task, expected_sample_time));
      scheduled_simulations_.push_back(simulation);
    }
    /*else
    {
      ruler::PreemptiveTaskPtr preemptive_task(new ruler::PreemptiveTask(id,
    name, expected_start, expected_end, waypoints));
    }*/
  }
}

void TaskSimulationNode::controlLoop()
{
  for (iterator it(scheduled_simulations_.begin()); it != scheduled_simulations_.end(); it++)
  {
    ruler::TaskSimulationPtr simulation(*it);
    simulation->update();
  }
}
}
