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
ContinuousReusableResource::ContinuousReusableResource(
    const std::string& id, const std::string& name,
    const utilities::ContinuousSignalType& capacity,
    const utilities::ContinuousSignalType& initial_level,
    const ros::Duration& latence)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(
          id, name, capacity, initial_level, latence)
{
}

ContinuousReusableResource::ContinuousReusableResource(
    const ruler_msgs::Resource& msg)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(msg)
{
}

ContinuousReusableResource::ContinuousReusableResource(
    const ContinuousReusableResource& resource)
    : ReusableResource<utilities::ContinuousSignalType>::ReusableResource(
          resource)
{
}

ContinuousReusableResource::~ContinuousReusableResource() {}
}
