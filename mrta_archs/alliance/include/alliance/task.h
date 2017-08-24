#ifndef _ALLIANCE_TASK_H_
#define _ALLIANCE_TASK_H_

#include <utilities/has_name.h>

namespace alliance
{
class Task : public utilities::HasName
{
public:
  Task(std::string id, std::string name);
  Task(const Task& task);
  virtual ~Task();
private:
};
}

#endif // _ALLIANCE_TASK_H_
