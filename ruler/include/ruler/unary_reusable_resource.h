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
  UnaryReusableResource(std::string id, std::string name,
                        bool initial_level = true,
                        ros::Duration latence = ros::Duration(0.0));
  UnaryReusableResource(std::string id, std::string name,
                        utilities::UnarySignalType initial_level,
                        ros::Duration latence = ros::Duration(0.0));
  UnaryReusableResource(const UnaryReusableResource& resource);
  virtual ~UnaryReusableResource();
  virtual void require(Task* task);
};
}

#endif // _RULER_UNARY_REUSABLE_RESOURCE_H_
