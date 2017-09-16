/**
 *  This header file defines the ResourceInterface abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_INTERFACE_H_
#define _RULER_RESOURCE_INTERFACE_H_

#include "ruler/task_event.h"
#include "ruler_msgs/Resource.h"
#include "utilities/observer.h"
#include "utilities/ros_message_converter.h"
#include "utilities/signal_types.h"

namespace ruler
{
class ResourceInterface : public utilities::Observer,
                          public utilities::ROSMessageConverter<ruler_msgs::Resource>
{
public:
  virtual ~ResourceInterface();
  virtual bool isConsumable() const = 0;
  virtual bool isReusable() const = 0;
  virtual bool isContinuous() const = 0;
  virtual bool isDiscrete() const = 0;
  virtual bool isUnary() const = 0;
  virtual utilities::SignalTypeEnum getSignalType() const = 0;
  using Observer::operator==;
  virtual bool operator==(const ruler_msgs::Resource& msg) const;

protected:
  ResourceInterface(const std::string& id);
  ResourceInterface(const ResourceInterface& resource);
};

typedef boost::shared_ptr<ResourceInterface> ResourceInterfacePtr;
typedef boost::shared_ptr<ResourceInterface const> ResourceInterfaceConstPtr;
}

#endif // _RULER_RESOURCE_INTERFACE_H_
