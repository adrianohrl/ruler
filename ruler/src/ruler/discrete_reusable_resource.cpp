/**
 *  This header file implements the DiscreteReusableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/discrete_reusable_resource.h"

namespace ruler
{
DiscreteReusableResource::DiscreteReusableResource(std::string type,
                                                   std::string name,
                                                   long capacity,
                                                   long initial_level,
                                                   ros::Duration latence)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          type, name, utilities::DiscreteSignalType(capacity),
          utilities::DiscreteSignalType(initial_level), latence)
{
}

DiscreteReusableResource::DiscreteReusableResource(
    std::string type, std::string name, utilities::DiscreteSignalType capacity,
    utilities::DiscreteSignalType initial_level, ros::Duration latence)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          type, name, capacity, initial_level, latence)
{
}

DiscreteReusableResource::DiscreteReusableResource(
    const DiscreteReusableResource& resource)
    : ReusableResource<utilities::DiscreteSignalType>::ReusableResource(
          resource)
{
}

DiscreteReusableResource::~DiscreteReusableResource() {}
}
