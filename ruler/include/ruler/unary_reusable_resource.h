/**
 *  This header file defines the UnaryReusableResource class, which is based on
 *the ReusableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_UNARY_REUSABLE_RESOURCE_H_
#define _RULER_UNARY_REUSABLE_RESOURCE_H_

#include "ruler/reusable_resource.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
class UnaryReusableResource
    : public ReusableResource<utilities::UnarySignalType>
{
public:
  UnaryReusableResource(const std::string& id, const std::string& name,
                        bool initial_level = true,
                        const ros::Duration& latence = ros::Duration());
  UnaryReusableResource(const std::string& id, const std::string& name,
                        const utilities::UnarySignalType& initial_level,
                        const ros::Duration& latence = ros::Duration());
  UnaryReusableResource(const ruler_msgs::Resource& msg);
  UnaryReusableResource(const UnaryReusableResource& resource);
  virtual ~UnaryReusableResource();
  virtual void require(const TaskPtr& task, double d0 = 0.0,
                       double df = INFINITY);
};

typedef boost::shared_ptr<UnaryReusableResource> UnaryReusableResourcePtr;
typedef boost::shared_ptr<UnaryReusableResource const>
    UnaryReusableResourceConstPtr;
}

#endif // _RULER_UNARY_REUSABLE_RESOURCE_H_
