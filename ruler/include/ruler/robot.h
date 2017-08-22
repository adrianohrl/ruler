/**
 *  This header file defines the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_ROBOT_H_
#define _RULER_ROBOT_H_

#include "ruler/resource_interface.h"
#include "utilities/has_id.h"

namespace ruler
{
class Robot : public utilities::HasId<std::string>
{
public:
  Robot(std::string id, std::string name, std::list<ResourceInterface*> resources = std::list<ResourceInterface*>());
  Robot(const Robot& robot);
  virtual ~Robot();
  std::string getName() const;
  std::list<ResourceInterface*> getResources() const;
  void addResource(ResourceInterface* resource);
  bool contains(const ResourceInterface& resource) const;

private:
  std::string name_;
  std::list<ResourceInterface*> resources_;
};
}

#endif // _RULER_ROBOT_H_
