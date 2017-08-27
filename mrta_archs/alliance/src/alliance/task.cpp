#include "alliance/task.h"

namespace alliance
{
Task::Task(const std::string& id, const std::string& name)
    : HasName::HasName(name, id)
{
}

Task::Task(const Task& task) : HasName::HasName(task) {}

Task::~Task() {}
}
