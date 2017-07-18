/**
 *  This header file defines the EventTypeEnum enumerateds and the EventTypes
 *helper class, which is based on the EnumConverter abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _EVENT_TYPE_H_
#define _EVENT_TYPE_H_

#include "utilities/enum_converter.h"

namespace ruler
{
namespace types
{
enum EventTypeEnum
{
  STARTED,
  INTERRUPTED,
  RESUMED,
  FINISHED
};
}

typedef types::EventTypeEnum EventTypeEnum;

class EventTypes : public utilities::EnumConverter<EventTypeEnum>
{
public:
  EventTypes(EventTypeEnum enumarated);
  virtual ~EventTypes();
  virtual EventTypeEnum getEnumerated(int id) const;
  virtual EventTypeEnum getEnumerated(std::string name) const;
  virtual int getId(std::string name) const;
  virtual int getId(E enumerated) const;
  virtual std::string str(E enumerated) const;
  virtual const char* c_str(E enumerated) const;

  static EventTypeEnum toEnumerated(int code);
  static EventType toEnumerated(std::string name);
  static int toCode(EventTypeEnum enumerated);
  static std::string str(EventTypeEnum enumerated);
  static const char* c_str(EventTypeEnum enumerated);
  static EventTypeEnum getDefault();
  static std::vector<EventTypeEnum> getAll();
};
}

#endif // _EVENT_TYPE_H_
