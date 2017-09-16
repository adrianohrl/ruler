/**
 *  This header file defines the DiscreteConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_DISCRETE_CONSUMABLE_RESOURCE_H_
#define _RULER_DISCRETE_CONSUMABLE_RESOURCE_H_

#include "ruler/consumable_resource.h"
#include "utilities/discrete_signal_type.h"

namespace ruler
{
class DiscreteConsumableResource
    : public ConsumableResource<utilities::DiscreteSignalType>
{
public:
  DiscreteConsumableResource(const std::string& id, const std::string& name,
                             long capacity, long initial_level = 0,
                             const ros::Duration& latence = ros::Duration());
  DiscreteConsumableResource(const std::string& id, const std::string& name,
                             const utilities::DiscreteSignalType& capacity,
                             const utilities::DiscreteSignalType& initial_level,
                             const ros::Duration& latence = ros::Duration());
  DiscreteConsumableResource(const ruler_msgs::Resource& msg);
  DiscreteConsumableResource(const DiscreteConsumableResource& resource);
  virtual ~DiscreteConsumableResource();
};

typedef boost::shared_ptr<DiscreteConsumableResource>
    DiscreteConsumableResourcePtr;
typedef boost::shared_ptr<DiscreteConsumableResource const>
    DiscreteConsumableResourceConstPtr;
}

#endif // _RULER_DISCRETE_CONSUMABLE_RESOURCE_H_
