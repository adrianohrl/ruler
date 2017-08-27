#ifndef _ALLIANCE_TASK_H_
#define _ALLIANCE_TASK_H_

#include <utilities/has_name.h>

namespace alliance
{
class Task : public utilities::HasName
{
public:
  Task(const std::string& id, const std::string& name);
  Task(const Task& task);
  virtual ~Task();
};
}

#endif // _ALLIANCE_TASK_H_
