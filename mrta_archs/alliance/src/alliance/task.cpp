#include "alliance/task.h"

namespace alliance
{
Task::Task(std::string id, std::string name)
  : HasName::HasName(name, id)
{

}

Task::Task(const Task &task)
  : HasName::HasName(task)
{

}

Task::~Task()
{

}
}
