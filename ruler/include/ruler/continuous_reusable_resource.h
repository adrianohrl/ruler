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
  ContinuousReusableResource(const std::string& id, const std::string& name, double capacity,
                             double initial_level = 0.0,
                             const ros::Duration& latence = ros::Duration());
  ContinuousReusableResource(const std::string& id, const std::string& name,
                             const utilities::ContinuousSignalType& capacity,
                             const utilities::ContinuousSignalType& initial_level,
                             const ros::Duration& latence = ros::Duration());
  ContinuousReusableResource(const ruler_msgs::Resource& msg);
  ContinuousReusableResource(const ContinuousReusableResource& resource);
  virtual ~ContinuousReusableResource();
  using ReusableResource<utilities::ContinuousSignalType>::require;
  virtual void require(const TaskPtr& task, double quantity, double d0 = 0.0);
};

typedef boost::shared_ptr<ContinuousReusableResource>
    ContinuousReusableResourcePtr;
typedef boost::shared_ptr<ContinuousReusableResource const>
    ContinuousReusableResourceConstPtr;
}

#endif // _RULER_CONTINUOUS_REUSABLE_RESOURCE_H_
