#include "alliance/robot_interface.h"
#include <utilities/exception.h>

namespace alliance
{
RobotInterface::RobotInterface(const std::string& id, const std::string& name)
    : HasName::HasName(name, id), active_behaviour_set_(NULL)
{
}

RobotInterface::~RobotInterface()
{
  active_behaviour_set_ = NULL;
  std::list<BehaviourSetInterface*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

std::list<BehaviourSetInterface *> RobotInterface::getBehaviourSets() const
{
  return behaviour_sets_;
}

Task* RobotInterface::getExecutingTask() const
{
  return active_behaviour_set_ ? active_behaviour_set_->getTask() : NULL;
}

bool RobotInterface::isIdle() const
{
  return !active_behaviour_set_;
}

void RobotInterface::addBehaviourSet(BehaviourSetInterface* behaviour_set)
{
  if (!behaviour_set)
  {
    ROS_ERROR_STREAM("The given robot's behaviour set must not be null.");
    return;
  }
  if (contains(*behaviour_set))
  {
    throw utilities::Exception("This behaviour set already exists.");
  }
  behaviour_sets_.push_back(behaviour_set);
}

bool RobotInterface::contains(const BehaviourSetInterface& behaviour_set) const
{
  std::list<BehaviourSetInterface*>::const_iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (**it == behaviour_set)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
