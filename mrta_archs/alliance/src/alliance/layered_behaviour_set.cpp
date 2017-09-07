#include "alliance/behaved_robot.h"
#include "alliance/layered_behaviour_set.h"
#include <utilities/exception.h>

namespace alliance
{
LayeredBehaviourSet::LayeredBehaviourSet(const BehavedRobotPtr& robot,
                                         const TaskPtr& task,
                                         const ros::Duration& buffer_horizon,
                                         const ros::Duration& timeout_duration)
    : BehaviourSetInterface<BehavedRobot>::
          BehaviourSetInterface(robot, task, buffer_horizon, timeout_duration),
      BeaconSignalObserver::BeaconSignalObserver(robot->getId() + "/" +
                                                 task->getId()),
      loader_("alliance", "alliance::Layer")
{
  std::list<std::string> layers(task->getNeededLayers());
  std::list<std::string>::const_iterator it(layers.begin());
  LayerPtr layer;
  while (it != layers.end())
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
    it++;
  }
}

LayeredBehaviourSet::~LayeredBehaviourSet() {}

void LayeredBehaviourSet::process()
{
  std::list<LayerPtr>::iterator it(layers_.begin());
  while (it != layers_.end())
  {
    LayerPtr layer(*it);
    layer->process();
    it++;
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
  std::list<LayerPtr>::const_iterator it(layers_.begin());
  while (it != layers_.end())
  {
    LayerPtr layer(*it);
    if (layer->getName() == layer_name)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
