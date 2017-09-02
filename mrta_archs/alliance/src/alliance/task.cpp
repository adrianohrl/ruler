#include "alliance/task.h"

namespace alliance
{
Task::Task(const std::string& id, const std::string& name)
    : HasName::HasName(name, id)
{
}

Task::Task(const Task& task) : HasName::HasName(task) {}

Task::~Task() {}

std::list<std::string> Task::getNeededLayers() const
{
  return needed_layers_;
}

void Task::addNeededLayer(const std::string &layer_name)
{
  if (!contains(layer_name))
  {
    needed_layers_.push_back(layer_name);
  }
}

bool Task::contains(const std::string &layer_name) const
{
  std::list<std::string>::const_iterator it(needed_layers_.begin());
  while (it != needed_layers_.end())
  {
    if (*it == layer_name)
    {
      return true;
    }
    it++;
  }
  return false;

}
}
