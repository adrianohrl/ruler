/**
 *  This header file implements the ContinuousReusableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/continuous_reusable_resource.h"

namespace ruler
{
ContinuousReusableResource::ContinuousReusableResource(std::string id,
                                                       std::string name,
                                                       double capacity,
                                                       double initial_level,
                                                       ros::Duration latence)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(
          id, name, utilities::ContinuousSignalType(capacity),
          utilities::ContinuousSignalType(initial_level), latence)
{
}
ContinuousReusableResource::ContinuousReusableResource(
    std::string id, std::string name, utilities::ContinuousSignalType capacity,
    utilities::ContinuousSignalType initial_level, ros::Duration latence)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(
          id, name, capacity, initial_level, latence)
{
}

ContinuousReusableResource::ContinuousReusableResource(
    const ContinuousReusableResource& resource)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(
          resource)
{
}

ContinuousReusableResource::~ContinuousReusableResource() {}

void ContinuousReusableResource::require(Task* task, double quantity)
{
  ReusableResource<utilities::ContinuousSignalType>::require(
      task, utilities::ContinuousSignalType(quantity));
}
}