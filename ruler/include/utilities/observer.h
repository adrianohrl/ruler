/**
 *  This header file defines and implements the Observer abstract class of the
 *Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_OBSERVER_H_
#define _UTILITIES_OBSERVER_H_

#include "utilities/has_id.h"
#include "utilities/event.h"

namespace utilities
{
class Observer : public HasId<std::string>
{
public:
  virtual ~Observer();
  virtual void update(Event* event) = 0;

protected:
  Observer(std::string id);
  Observer(const Observer& observer);
};
}

#endif // _UTILITIES_OBSERVER_H_
