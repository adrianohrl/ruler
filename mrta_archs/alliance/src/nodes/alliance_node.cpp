#include "nodes/alliance_node.h"

namespace nodes
{
AllianceNode::AllianceNode(ros::NodeHandle* nh, float loop_rate)
    : ROSNode::ROSNode(nh, loop_rate),
      BeaconSignalSubject::BeaconSignalSubject(ros::this_node::getName()),
      robot_(NULL), started_broadcasting_(false)
{
  beacon_signal_pub_ =
      nh->advertise<alliance_msgs::BeaconSignal>("/alliance/beacon_signal", 10);
  beacon_signal_sub_ = nh->subscribe("/alliance/beacon_signal", 100,
                                     &AllianceNode::beaconSignalCallback, this);
}

AllianceNode::~AllianceNode()
{
  beacon_signal_pub_.shutdown();
  beacon_signal_sub_.shutdown();
  broadcast_timer_.stop();
  if (robot_)
  {
    delete robot_;
    robot_ = NULL;
  }
}

void AllianceNode::readParameters()
{
  ros::NodeHandle pnh("~");
  std::string id, name;
  pnh.param("id", id, std::string(""));
  pnh.param("name", name, std::string(""));
  if (id.empty())
  {
    ROSNode::shutdown("Not found robot id as a ROS parameter.");
    return;
  }
  if (!std::equal(id.rbegin(), id.rend(),
                  ros::this_node::getNamespace().rbegin()))
  {
    ROSNode::shutdown("Invalid ROS namespace. It must end with '" + id + "'.");
    return;
  }
  robot_ = new alliance::Robot(id, name);
  double broadcast_rate;
  pnh.param("broadcast_rate", broadcast_rate, 0.0);
  if (broadcast_rate <= 0.0)
  {
    ROSNode::shutdown(
        "The robot's inter communication broadcast rate must be positive.");
    return;
  }
  robot_->setBroadcastRate(ros::Rate(broadcast_rate));
  double timeout_duration;
  pnh.param("timeout_duration", timeout_duration, 0.0);
  if (timeout_duration < 0.0)
  {
    ROS_WARN(
        "The robot's inter communication timeout duration must be positive.");
    timeout_duration = 0.0;
  }
  robot_->setTimeoutDuration(ros::Duration(timeout_duration));
  pnh = ros::NodeHandle("~/behaviour_sets");
  int size;
  pnh.param("size", size, 0);
  for (int i(0); i < size; i++)
  {
    std::stringstream ss;
    ss << "behaviour_set" << i << "/";
    pnh.param(ss.str() + "task/id", id, std::string(""));
    if (id.empty())
    {
      ROS_ERROR("The behaviour set's task id must not be empty.");
      continue;
    }
    pnh.param(ss.str() + "task/name", name, std::string(""));
    alliance::Task* task = new alliance::Task(id, name.empty() ? id : name);
    alliance::BehaviourSet* behaviour_set =
        new alliance::BehaviourSet(robot_, task);
    ss << "motivational_behaviour/";
    double threshold;
    pnh.param(ss.str() + "threshold", threshold, 0.0);
    if (threshold <= 0.0)
    {
      ROS_ERROR("The behaviour set's motivational behaviour's threshold must "
                "be positive.");
      continue;
    }
    behaviour_set->setActivationThreshold(threshold);
    double yielding_delay, giving_up_delay;
    pnh.param(ss.str() + "acquiescence/yielding_delay", yielding_delay, 0.0);
    if (yielding_delay < 0.0)
    {
      ROS_WARN("The robot's yielding delay must not be negative.");
      yielding_delay = 0.0;
    }
    pnh.param(ss.str() + "acquiescence/giving_up_delay", giving_up_delay, 0.0);
    if (giving_up_delay < 0.0)
    {
      ROS_WARN("The robot's yielding delay must not be negative.");
      giving_up_delay = 0.0;
    }
    behaviour_set->setAcquiescence(ros::Duration(yielding_delay),
                                   ros::Duration(giving_up_delay));
    double fast_rate;
    pnh.param(ss.str() + "impatience/fast_rate", fast_rate, 0.0);
    if (fast_rate <= 0.0)
    {
      ROS_ERROR("The robot's impatience fast rate must be positive.");
      continue;
    }
    behaviour_set->setImpatience(fast_rate);
    robot_->addBehaviourSet(behaviour_set);
  }
  if (robot_->getBehaviourSets().empty())
  {
    ROSNode::shutdown("None behaviour set was imported to " + robot_->str() +
                      " robot.");
  }
}

void AllianceNode::init()
{
  /** registering beacon signal message observers **/
  std::list<alliance::BehaviourSet*> behaviour_sets(robot_->getBehaviourSets());
  std::list<alliance::BehaviourSet*>::iterator it(behaviour_sets.begin());
  while (it != behaviour_sets.end())
  {
    alliance::MotivationalBehaviour* motivational_behaviour =
        ((alliance::BehaviourSet*)*it)->getMotivationalBehaviour();
    BeaconSignalSubject::registerObserver(
        motivational_behaviour->getInterCommunication());
    it++;
  }
  /** creating robot broadcast timer **/
  broadcast_timer_ = ROSNode::getNodeHandle()->createTimer(
      robot_->getBroadcastRate().expectedCycleTime(),
      &AllianceNode::broadcastTimerCallback, this, false, false);
}

void AllianceNode::controlLoop()
{
  robot_->process();
  if (robot_->isActive() && !started_broadcasting_)
  {
    ROS_WARN_STREAM("Starting " << *robot_ << " broadcast timer.");
    broadcast_timer_.start();
    started_broadcasting_ = true;
  }
}

void AllianceNode::beaconSignalCallback(const alliance_msgs::BeaconSignal& msg)
{
  BeaconSignalSubject::notify(msg);
}

void AllianceNode::broadcastTimerCallback(const ros::TimerEvent& event)
{
  if (!robot_->getExecutingTask())
  {
    ROS_WARN_STREAM("Stoping " << *robot_ << " broadcast timer.");
    broadcast_timer_.stop();
    started_broadcasting_ = false;
    return;
  }
  alliance_msgs::BeaconSignal msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = robot_->getId();
  msg.task_id = robot_->getExecutingTask()->getId();
  beacon_signal_pub_.publish(msg);
}
}
