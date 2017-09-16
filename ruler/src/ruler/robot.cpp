/**
 *  This source file implements the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/robot.h"

namespace ruler
{
Robot::Robot(const std::string& id, const std::string& name,
             const std::list<ResourceInterfacePtr>& resources)
    : HasName::HasName(name, id), resources_(resources)
{
}

Robot::Robot(const Robot& robot)
    : HasName::HasName(robot), resources_(robot.resources_)
{
}

Robot::~Robot()
{
}

std::list<ResourceInterfacePtr> Robot::getResources() const
{
  return resources_;
}

void Robot::addResource(const ResourceInterfacePtr& resource)
{
  if (contains(*resource))
  {
    throw utilities::Exception("Unable to add the " + resource->str() +
                               " resource. The " + str() +
                               " robot already has it.");
  }
  resources_.push_back(resource);
}

bool Robot::contains(const ResourceInterface& resource) const
{
  for (const_iterator it(resources_.begin()); it != resources_.end(); it++)
  {
    if (**it == resource)
    {
      return true;
    }
  }
  return false;
}

std::size_t Robot::size() const { return resources_.size(); }

bool Robot::empty() const { return resources_.empty(); }

Robot::iterator Robot::begin() { return resources_.begin(); }

Robot::const_iterator Robot::begin() const { return resources_.begin(); }

Robot::iterator Robot::end() { return resources_.end(); }

Robot::const_iterator Robot::end() const { return resources_.end(); }
}
