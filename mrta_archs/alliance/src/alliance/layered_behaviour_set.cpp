#include "alliance/behaved_robot.h"
#include "alliance/layered_behaviour_set.h"
#include <utilities/exception.h>

namespace alliance
{
LayeredBehaviourSet::LayeredBehaviourSet(BehavedRobot* robot, Task* task)
    : BehaviourSetInterface::BehaviourSetInterface(robot, task),
      loader_("alliance", "alliance::Layer")
{
}

LayeredBehaviourSet::~LayeredBehaviourSet() {}

void LayeredBehaviourSet::process()
{
  std::list<boost::shared_ptr<Layer> >::iterator it(layers_.begin());
  while (it != layers_.end())
  {
    ((boost::shared_ptr<Layer>)*it)->process();
    it++;
  }
}

void LayeredBehaviourSet::addLayer(const std::string& layer_name)
{
  try
  {
    boost::shared_ptr<Layer> layer(loader_.createInstance(layer_name.c_str()));
    layer->initialize(getId() + "/" + layer_name);
    layers_.push_back(layer);
    ROS_DEBUG_STREAM("Loaded " << layer_name << " layer plugin to the " << *this
                               << " behaviour set.");
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

bool LayeredBehaviourSet::contains(const std::string& layer_name) const
{
  std::list<boost::shared_ptr<Layer> >::const_iterator it(layers_.begin());
  while (it != layers_.end())
  {
    boost::shared_ptr<Layer> layer = *it;
    if (layer->getName() == layer_name)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
