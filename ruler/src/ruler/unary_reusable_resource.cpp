/**
 *  This header file implements the UnaryReusableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/unary_reusable_resource.h"

namespace ruler
{

UnaryReusableResource::UnaryReusableResource(std::string type, std::string name,
                                             bool initial_level,
                                             ros::Duration latence)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(
          type, name, utilities::UnarySignalType(true),
          utilities::UnarySignalType(initial_level), latence)
{
}
UnaryReusableResource::UnaryReusableResource(
    std::string type, std::string name,
    utilities::UnarySignalType initial_level, ros::Duration latence)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(
          type, name, utilities::UnarySignalType(true), initial_level, latence)
{
}

UnaryReusableResource::UnaryReusableResource(
    const UnaryReusableResource& resource)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(resource)
{
}

UnaryReusableResource::~UnaryReusableResource() {}
}
