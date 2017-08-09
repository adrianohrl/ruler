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
  DiscreteConsumableResource(const ruler_msgs::Resource& msg);
  DiscreteConsumableResource(std::string id, std::string name,
                             long capacity, long initial_level = 0,
                             ros::Duration latence = ros::Duration(0.0));
  DiscreteConsumableResource(std::string id, std::string name,
                             utilities::DiscreteSignalType capacity,
                             utilities::DiscreteSignalType initial_level,
                             ros::Duration latence = ros::Duration(0.0));
  DiscreteConsumableResource(const DiscreteConsumableResource& resource);
  virtual ~DiscreteConsumableResource();
};
}

#endif // _RULER_DISCRETE_CONSUMABLE_RESOURCE_H_
