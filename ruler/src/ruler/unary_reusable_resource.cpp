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
UnaryReusableResource::UnaryReusableResource(const std::string& id,
                                             const std::string& name,
                                             bool initial_level,
                                             const ros::Duration& latence)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(
          id, name, utilities::UnarySignalType(true),
          utilities::UnarySignalType(initial_level), latence)
{
}
UnaryReusableResource::UnaryReusableResource(
    const std::string& id, const std::string& name,
    const utilities::UnarySignalType& initial_level,
    const ros::Duration& latence)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(
          id, name, utilities::UnarySignalType(true), initial_level, latence)
{
}

UnaryReusableResource::UnaryReusableResource(const ruler_msgs::Resource& msg)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(msg)
{
  utilities::SignalTypeEnum signal_type(
      utilities::SignalTypes::toEnumerated(msg.signal_type));
  if (signal_type != utilities::signal_types::UNARY)
  {
    throw utilities::Exception(
        "Not an unary signal type resource ros message.");
  }
}

UnaryReusableResource::UnaryReusableResource(
    const UnaryReusableResource& resource)
    : ReusableResource<utilities::UnarySignalType>::ReusableResource(resource)
{
}

UnaryReusableResource::~UnaryReusableResource() {}

void UnaryReusableResource::require(const TaskPtr& task, double d0, double df)
{
  /*ReusableResource<utilities::UnarySignalType>::require(
      task, new utilities::StepFunction());*/
}
}
