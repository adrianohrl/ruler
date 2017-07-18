/**
 *  This header file implements the EventTypes helper class, which is based on
 *the EnumConverter abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/event_type.h"

namespace ruler
{

EventTypes::EventTypes(EventTypeEnum enumarated) : EnumConverter(enumarated) {}

EventTypes::~EventTypes() {}

EventTypeEnum EventTypes::getEnumerated(int code) const
{
  return EventTypes::toEnumerated(code);
}

EventTypeEnum EventTypes::getEnumerated(std::string name) const
{
  return EventTypes::toEnumerated(name);
}

int EventTypes::getCode(std::string name) const
{
  return EventTypes::toCode(EventTypes::toEnumerated(name));
}

int EventTypes::getCode(EventTypeEnum enumerated) const
{
  return EventTypes::toCode(enumerated);
}

std::string EventTypes::str(EventTypeEnum enumerated) const
{
  return EventTypes::toString(enumerated);
}

const char* EventTypes::c_str(EventTypeEnum enumerated) const {}

EventTypeEnum EventTypes::toEnumerated(int code)
{
  EventTypeEnum enumerated;
  switch (code)
  {
  case 0:
    enumerated = types::STARTED;
    break;
  case 1:
    enumerated = types::INTERRUPTED;
    break;
  case 2:
    enumerated = types::RESUMED;
    break;
  case 3:
    enumerated = types::FINISHED;
    break;
  default:
    enumerated = EventTypes::getDefault();
  }
  return enumerated;
}

EventTypeEnum EventTypes::toEnumerated(std::string name)
{
  EventTypeEnum enumerated;
  if (name == "STARTED" || name == "Started" || name == "started")
  {
    enumerated = types::STARTED;
  }
  else if (name == "INTERRUPTED" || name == "Interrupted" ||
           name == "interrupted")
  {
    enumerated = types::INTERRUPTED;
  }
  else if (name == "RESUMED" || name == "Resumed" || name == "resumed")
  {
    enumerated = types::RESUMED;
  }
  else if (name == "FINISHED" || name == "Finished" || name == "finished")
  {
    enumerated = types::FINISHED;
  }
  else
  {
    enumerated = EventTypes::getDefault();
  }
  return enumerated;
}

int EventTypes::toCode(EventTypeEnum enumerated)
{
  int code;
  switch (enumerated)
  {
  case types::STARTED:
    code = 0;
    break;
  case types::INTERRUPTED:
    code = 1;
    break;
  case types::RESUMED:
    code = 2;
    break;
  case types::FINISHED:
    code = 3;
    break;
  default:
    code = EventTypes::toCode(EventTypes::getDefault());
  }
  return code;
}

std::string EventTypes::toString(EventTypeEnum enumerated)
{
  std::string name;
  switch (enumerated)
  {
  case types::STARTED:
    name = "STARTED";
    break;
  case types::INTERRUPTED:
    name = "INTERRUPTED";
    break;
  case types::RESUMED:
    name = "RESUMED";
    break;
  case types::FINISHED:
    name = "FINISHED";
    break;
  default:
    name = EventTypes::toString(EventTypes::getDefault());
  }
  return name;
}

const char* EventTypes::toCString(EventTypeEnum enumerated)
{
  return EventTypes::toString(enumerated).c_str();
}

EventTypeEnum EventTypes::getDefault() { return types::STARTED; }

std::vector<EventTypeEnum> EventTypes::getAll()
{
  std::vector<EventTypeEnum> types;
  types.push_back(types::STARTED);
  types.push_back(types::INTERRUPTED);
  types.push_back(types::RESUMED);
  types.push_back(types::FINISHED);
  return types;
}
}
