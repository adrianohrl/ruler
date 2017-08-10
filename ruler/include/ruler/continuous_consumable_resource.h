/**
 *  This header file defines the ContinuousConsumableResource class, which is
 *based on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONTINUOUS_CONSUMABLE_RESOURCE_H_
#define _RULER_CONTINUOUS_CONSUMABLE_RESOURCE_H_

#include "ruler/consumable_resource.h"
#include "utilities/continuous_signal_type.h"

namespace ruler
{
class ContinuousConsumableResource
    : public ConsumableResource<utilities::ContinuousSignalType>
{
public:
  ContinuousConsumableResource(std::string id, std::string name,
                               double capacity, double initial_level = 0.0,
                               ros::Duration latence = ros::Duration(0.0));
  ContinuousConsumableResource(std::string id, std::string name,
                               utilities::ContinuousSignalType capacity,
                               utilities::ContinuousSignalType initial_level,
                               ros::Duration latence = ros::Duration(0.0));
  ContinuousConsumableResource(const ruler_msgs::Resource& msg);
  ContinuousConsumableResource(const ContinuousConsumableResource& resource);
  virtual ~ContinuousConsumableResource();
};
}

#endif // _RULER_CONTINUOUS_CONSUMABLE_RESOURCE_H_
