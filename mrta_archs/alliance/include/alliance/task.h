#ifndef _ALLIANCE_TASK_H_
#define _ALLIANCE_TASK_H_

#include <boost/shared_ptr.hpp>
#include <list>
#include <utilities/has_name.h>

namespace alliance
{
class Task : public utilities::HasName
{
public:
  Task(const std::string& id, const std::string& name);
  Task(const Task& task);
  virtual ~Task();
  std::list<std::string> getNeededLayers() const;
  void addNeededLayer(const std::string& layer_name);

private:
  std::list<std::string> needed_layers_;
  bool contains(const std::string& layer_name) const;
};

typedef boost::shared_ptr<Task> TaskPtr;
typedef boost::shared_ptr<Task const> TaskConstPtr;
}

#endif // _ALLIANCE_TASK_H_
