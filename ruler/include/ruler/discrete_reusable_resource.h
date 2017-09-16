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
  DiscreteReusableResource(const std::string& id, const std::string& name,
                           long capacity, long initial_level = 0,
                           const ros::Duration& latence = ros::Duration());
  DiscreteReusableResource(const std::string& id, const std::string& name,
                           const utilities::DiscreteSignalType& capacity,
                           const utilities::DiscreteSignalType& initial_level,
                           const ros::Duration& latence = ros::Duration());
  DiscreteReusableResource(const ruler_msgs::Resource& msg);
  DiscreteReusableResource(const DiscreteReusableResource& resource);
  virtual ~DiscreteReusableResource();
  using ReusableResource<utilities::DiscreteSignalType>::require;
  virtual void require(const TaskPtr& task, long quantity, double d0 = 0.0);
};

typedef boost::shared_ptr<DiscreteReusableResource> DiscreteReusableResourcePtr;
typedef boost::shared_ptr<DiscreteReusableResource const>
    DiscreteReusableResourceConstPtr;
}

#endif // _RULER_DISCRETE_REUSABLE_RESOURCE_H_
