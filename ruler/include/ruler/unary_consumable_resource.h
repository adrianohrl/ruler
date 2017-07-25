/**
 *  This header file defines the UnaryConsumableResource class, which is based
 *on the ConsumableResource class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_UNARY_CONSUMABLE_RESOURCE_H_
#define _RULER_UNARY_CONSUMABLE_RESOURCE_H_

#include "ruler/consumable_resource.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
class UnaryConsumableResource
    : public ConsumableResource<utilities::UnarySignalType>
{
public:
  UnaryConsumableResource(std::string type, std::string name,
                          bool initial_level = true,
                          ros::Duration latence = ros::Duration(0.0));
  UnaryConsumableResource(std::string type, std::string name,
                          utilities::UnarySignalType initial_level,
                          ros::Duration latence = ros::Duration(0.0));
  UnaryConsumableResource(const UnaryConsumableResource& resource);
  virtual ~UnaryConsumableResource();
};
}

#endif // _RULER_UNARY_CONSUMABLE_RESOURCE_H_
