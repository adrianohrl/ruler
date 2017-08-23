/**
 * This source file implements the HasName class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/has_name.h"

namespace utilities
{
HasName::HasName(std::string name, std::string id)
    : HasId<std::string>::HasId(id.empty() ? name : id), name_(name)
{
}

HasName::HasName(const HasName& has_name)
    : HasId<std::string>::HasId(has_name), name_(has_name.name_)
{
}

HasName::~HasName() {}

std::string HasName::getName() const { return name_; }
}
