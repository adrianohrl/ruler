#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name,
                           const std::string& ns)
    : RobotInterface<LayeredBehaviourSet>::RobotInterface(id, name, ns),
      loader_("alliance", "alliance::Sensor")
{
}

BehavedRobot::~BehavedRobot() {}

void BehavedRobot::addSensor(const std::string& plugin_name,
                             const std::string& topic_name)
{
  try
  {
    SensorPtr sensor(loader_.createInstance(plugin_name.c_str()));
    sensor->initialize(ns_, plugin_name, ns_ + "/" + topic_name);
    addSensor(sensor);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s",
              ex.what());
  }
  if (contains(plugin_name))
  {
    throw utilities::Exception("This layer already exists.");
  }
}

void BehavedRobot::addSensor(const SensorPtr& sensor)
{
  sensors_.push_back(sensor);
  ROS_WARN_STREAM("Loaded " << sensor->getName() << " layer plugin to the "
                             << *this << " behaviour set.");
}

bool BehavedRobot::contains(const std::string& plugin_name) const
{
  for (sensors_const_iterator it(sensors_.begin()); it != sensors_.end(); it++)
  {
    SensorPtr sensor(*it);
    if (sensor->getName() == plugin_name)
    {
      return true;
    }
  }
  return false;
}
}
