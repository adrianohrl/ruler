#include "alliance/behaved_robot.h"
#include "alliance/layered_behaviour_set.h"
#include <utilities/exception.h>

namespace alliance
{
LayeredBehaviourSet::LayeredBehaviourSet(const BehavedRobotPtr& robot,
                                         const TaskPtr& task,
                                         const ros::Duration& buffer_horizon,
                                         const ros::Duration& timeout_duration)
    : BehaviourSetInterface<BehavedRobot>::BehaviourSetInterface(
          robot, task, buffer_horizon, timeout_duration),
      BeaconSignalObserver::BeaconSignalObserver(robot->getId() + "/" +
                                                 task->getId()),
      loader_("alliance", "alliance::Layer")
{
  LayerPtr layer;
  for (Task::const_iterator it(task->begin()); it != task->end(); it++)
  {
    try
    {
      layer = loader_.createInstance(it->c_str());
      ROS_DEBUG_STREAM("Loaded " << *it << " layer plugin to execute " << *task
                                 << " task.");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Could not load " << *it << ". " << ex.what());
    }
  }
}

LayeredBehaviourSet::~LayeredBehaviourSet() {}

void LayeredBehaviourSet::process()
{
  for (layers_iterator it(layers_.begin()); it != layers_.end(); it++)
  {
    LayerPtr layer(*it);
    layer->process();
  }
}

void LayeredBehaviourSet::addLayer(const std::string& layer_name)
{
  try
  {
    LayerPtr layer(loader_.createInstance(layer_name.c_str()));
    layer->initialize(getId() + "/" + layer_name);
    addLayer(layer);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }
  if (contains(layer_name))
  {
    throw utilities::Exception("This layer already exists.");
  }
}

void LayeredBehaviourSet::addLayer(const LayerPtr& layer)
{
  layers_.push_back(layer);
  ROS_DEBUG_STREAM("Loaded " << layer->getName() << " layer plugin to the "
                             << *this << " behaviour set.");
}

void LayeredBehaviourSet::update(
    const utilities::BeaconSignalEventConstPtr& event)
{
  if (!event->isRelated(*robot_) || !event->isRelated(*task_))
  {
    return;
  }
  setActive(true, event->getTimestamp());
}

bool LayeredBehaviourSet::contains(const std::string& layer_name) const
{
  for (layers_const_iterator it(layers_.begin()); it != layers_.end(); it++)
  {
    LayerPtr layer(*it);
    if (layer->getName() == layer_name)
    {
      return true;
    }
  }
  return false;
}
}
