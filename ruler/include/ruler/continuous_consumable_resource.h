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
  ContinuousConsumableResource(const std::string& id, const std::string& name,
                               double capacity, double initial_level = 0.0,
                               const ros::Duration& latence = ros::Duration());
  ContinuousConsumableResource(
      const std::string& id, const std::string& name,
      const utilities::ContinuousSignalType& capacity,
      const utilities::ContinuousSignalType& initial_level,
      const ros::Duration& latence = ros::Duration());
  ContinuousConsumableResource(const ruler_msgs::Resource& msg);
  ContinuousConsumableResource(const ContinuousConsumableResource& resource);
  virtual ~ContinuousConsumableResource();
};

typedef boost::shared_ptr<ContinuousConsumableResource>
    ContinuousConsumableResourcePtr;
typedef boost::shared_ptr<ContinuousConsumableResource const>
    ContinuousConsumableResourceConstPtr;
}

#endif // _RULER_CONTINUOUS_CONSUMABLE_RESOURCE_H_
