/**
 *  This header file defines and implements the Subject class of the Observer
 *Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SUBJECT_H_
#define _UTILITIES_SUBJECT_H_

#include <list>
#include <ros/common.h>
#include "utilities/observer.h"

namespace utilities
{
class Subject : public HasId<std::string>
{
public:
  virtual ~Subject();

protected:
  Subject(const std::string& id);
  Subject(const Subject& subject);
  void registerObserver(Observer* observer);
  void unregisterObserver(Observer* observer);
  void clearObservers();
  void notify(Event* event);
  bool empty() const;

private:
  std::list<Observer*> observers_;
};
}

#endif // _UTILITIES_SUBJECT_H_
