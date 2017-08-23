/**
 *  This source file implements and implements the Event class of
 *the Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/event.h"
#include "utilities/subject.h"

namespace utilities
{
Event::Event(Subject* subject, ros::Time timestamp)
    : subject_(subject), timestamp_(timestamp)
{
  if (!subject)
  {
    throw utilities::Exception("The event subject must not be NULL.");
  }
  if (timestamp.isZero())
  {
    throw utilities::Exception("The timestamp must not be zero.");
  }
}

Event::Event(const Event& event)
    : subject_(event.subject_), timestamp_(event.timestamp_)
{
}

Event::~Event() { subject_ = NULL; }

ros::Time Event::getTimestamp() const { return timestamp_; }

Subject* Event::getSubject() const { return subject_; }
}
