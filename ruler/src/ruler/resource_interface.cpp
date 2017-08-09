/**
 *  This header file defines the ResourceInterface abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/resource_interface.h"

namespace ruler
{

ResourceInterface::ResourceInterface(const std::string id)
    : Observer<TaskEvent>::Observer(id)
{
}

ResourceInterface::ResourceInterface(const ResourceInterface& resource)
    : Observer<TaskEvent>::Observer(resource)
{
}

ResourceInterface::~ResourceInterface() {}

bool ResourceInterface::operator==(const ruler_msgs::Resource& msg) const
{
  return getId() == msg.header.frame_id;
}
}
