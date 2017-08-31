/**
 *  This header file defines and implements the Event class of
 *the Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_EVENT_H_
#define _UTILITIES_EVENT_H_

#include <ros/time.h>
#include "utilities/exception.h"

namespace utilities
{
class Subject;

class Event
{
public:
  Event(Subject* subject, ros::Time timestamp = ros::Time::now());
  Event(const Event& event);
  virtual ~Event();
  ros::Time getTimestamp() const;
  Subject* getSubject() const;

private:
  const ros::Time timestamp_;
  Subject* subject_;
};
}

#endif // _UTILITIES_EVENT_H_
