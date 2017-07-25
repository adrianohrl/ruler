/**
 *  This header file defines the DiscreteReusableResource class, which is based
 *on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_DISCRETE_REUSABLE_RESOURCE_H_
#define _RULER_DISCRETE_REUSABLE_RESOURCE_H_

#include "ruler/reusable_resource.h"
#include "utilities/discrete_signal_type.h"

namespace ruler
{
class DiscreteReusableResource
    : public ReusableResource<utilities::DiscreteSignalType>
{
public:
  DiscreteReusableResource(std::string type, std::string name,
                           long capacity, long initial_level = 0,
                           ros::Duration latence = ros::Duration(0.0));
  DiscreteReusableResource(std::string type, std::string name,
                           utilities::DiscreteSignalType capacity,
                           utilities::DiscreteSignalType initial_level,
                           ros::Duration latence = ros::Duration(0.0));
  DiscreteReusableResource(const DiscreteReusableResource& resource);
  virtual ~DiscreteReusableResource();
};
}

#endif // _RULER_DISCRETE_REUSABLE_RESOURCE_H_
