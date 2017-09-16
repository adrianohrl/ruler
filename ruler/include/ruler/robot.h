/**
 *  This header file defines the Robot class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_ROBOT_H_
#define _RULER_ROBOT_H_

#include "ruler/resource_interface.h"
#include <utilities/has_name.h>

namespace ruler
{
class Robot : public utilities::HasName
{
public:
  typedef std::list<ResourceInterfacePtr>::iterator iterator;
  typedef std::list<ResourceInterfacePtr>::const_iterator const_iterator;
  Robot(const std::string& id, const std::string& name,
        const std::list<ResourceInterfacePtr>& resources =
            std::list<ResourceInterfacePtr>());
  Robot(const Robot& robot);
  virtual ~Robot();
  std::list<ResourceInterfacePtr> getResources() const;
  void addResource(const ResourceInterfacePtr& resource);
  bool contains(const ResourceInterface& resource) const;
  std::size_t size() const;
  bool empty() const;
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

private:
  std::list<ResourceInterfacePtr> resources_;
};

typedef boost::shared_ptr<Robot> RobotPtr;
typedef boost::shared_ptr<Robot const> RobotConstPtr;
}

#endif // _RULER_ROBOT_H_
