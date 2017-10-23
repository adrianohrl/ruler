/**
 *  This header file defines the UnaryConsumableResource class, which is based
 *on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_UNARY_CONSUMABLE_RESOURCE_H_
#define _RULER_UNARY_CONSUMABLE_RESOURCE_H_

#include "ruler/consumable_resource.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
class UnaryConsumableResource
    : public ConsumableResource<utilities::UnarySignalType>
{
protected:
  typedef utilities::functions::Function<utilities::UnarySignalType>::Ptr
      UnaryFunctionPtr;
  typedef utilities::functions::Function<utilities::UnarySignalType>::ConstPtr
      UnaryFunctionConstPtr;

public:
  UnaryConsumableResource(const std::string& id, const std::string& name,
                          const utilities::UnarySignalType& initial_level = true,
                          const ros::Duration& latence = ros::Duration());
  UnaryConsumableResource(const ruler_msgs::Resource& msg);
  UnaryConsumableResource(const UnaryConsumableResource& resource);
  virtual ~UnaryConsumableResource();
  virtual void consume(const TaskPtr& task, double d0 = 0.0,
                       double df = INFINITY);
  virtual void consume(const TaskPtr& task,
                       const UnaryFunctionPtr& quantity_function);
  virtual void produce(const TaskPtr& task, double d0 = 0.0,
                       double df = INFINITY);
  virtual void produce(const TaskPtr& task,
                       const UnaryFunctionPtr& quantity_function);
};

typedef boost::shared_ptr<UnaryConsumableResource> UnaryConsumableResourcePtr;
typedef boost::shared_ptr<UnaryConsumableResource const>
    UnaryConsumableResourceConstPtr;
}

#endif // _RULER_UNARY_CONSUMABLE_RESOURCE_H_
