/**
 * This header file defines the HasName class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_HAS_NAME_H_
#define _UTILITIES_HAS_NAME_H_

#include "utilities/has_id.h"

namespace utilities
{
class HasName : public HasId<std::string>
{
public:
  HasName(std::string name, std::string id = "");
  HasName(const HasName& has_name);
  virtual ~HasName();
  std::string getName() const;

private:
  std::string name_;
};
}

#endif // _UTILITIES_HAS_NAME_H_
