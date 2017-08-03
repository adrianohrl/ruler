/**
 *  This header file defines the ContinuousReusableResource class, which is
 *based on the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_CONTINUOUS_REUSABLE_RESOURCE_H_
#define _RULER_CONTINUOUS_REUSABLE_RESOURCE_H_

#include "ruler/reusable_resource.h"
#include "utilities/continuous_signal_type.h"

namespace ruler
{
class ContinuousReusableResource
    : public ReusableResource<utilities::ContinuousSignalType>
{
public:
  ContinuousReusableResource(std::string id, std::string name,
                             double capacity, double initial_level = 0.0,
                             ros::Duration latence = ros::Duration(0.0));
  ContinuousReusableResource(std::string id, std::string name,
                             utilities::ContinuousSignalType capacity,
                             utilities::ContinuousSignalType initial_level,
                             ros::Duration latence = ros::Duration(0.0));
  ContinuousReusableResource(const ContinuousReusableResource& resource);
  virtual ~ContinuousReusableResource();
  using ReusableResource<utilities::ContinuousSignalType>::require;
  virtual void require(Task *task, double quantity, double d0 = 0.0);
};
}

#endif // _RULER_CONTINUOUS_REUSABLE_RESOURCE_H_
