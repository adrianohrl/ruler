/**
 *  This source file implements the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/robot.h"

namespace ruler
{
Robot::Robot(std::string id, std::string name,
             std::list<ResourceInterface*> resources)
    : HasName::HasName(name, id), resources_(resources)
{
}

Robot::Robot(const Robot& robot)
    : HasName::HasName(robot), resources_(robot.resources_)
{
}

Robot::~Robot()
{
  std::list<ResourceInterface*>::iterator it(resources_.begin());
  while (it != resources_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

std::list<ResourceInterface*> Robot::getResources() const { return resources_; }

void Robot::addResource(ResourceInterface* resource)
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
  std::list<ResourceInterface*>::const_iterator it(resources_.begin());
  while (it != resources_.end())
  {
    if (**it == resource)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
