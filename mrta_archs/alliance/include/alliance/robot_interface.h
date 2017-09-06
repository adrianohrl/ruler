#ifndef _ALLIANCE_ROBOT_INTEFACE_H_
#define _ALLIANCE_ROBOT_INTEFACE_H_

#include "alliance/task.h"
#include <boost/enable_shared_from_this.hpp>
#include <list>
#include <ros/common.h>
#include <utilities/exception.h>
#include <utilities/has_name.h>

namespace alliance
{
template <typename R, typename BS>
class RobotInterface : public utilities::HasName,
                       public boost::enable_shared_from_this<R>
{

  typedef boost::shared_ptr<BS> BSPtr;

public:
  RobotInterface(const std::string& id, const std::string& name);
  virtual ~RobotInterface();
  void process();
  std::list<BSPtr> getBehaviourSets() const;
  TaskPtr getExecutingTask() const;
  bool isIdle() const;
  virtual void addBehaviourSet(const BSPtr& behaviour_set);

protected:
  BSPtr active_behaviour_set_;
  std::list<BSPtr> behaviour_sets_;
  bool contains(const BS& behaviour_set) const;
};

template <typename R, typename BS>
RobotInterface<R, BS>::RobotInterface(const std::string& id,
                                      const std::string& name)
    : HasName::HasName(name, id)
{
}

template <typename R, typename BS> RobotInterface<R, BS>::~RobotInterface() {}

template <typename R, typename BS> void RobotInterface<R, BS>::process()
{
  typename std::list<BSPtr>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BSPtr behaviour_set = *it;
    behaviour_set->preProcess();
    if (behaviour_set->isActive())
    {
      if (active_behaviour_set_ && *active_behaviour_set_ != *behaviour_set)
      {
        active_behaviour_set_->setActive(false);
      }
      active_behaviour_set_ = behaviour_set;
      ROS_WARN_STREAM("[BehavedRobot] active: " << *active_behaviour_set_);
      active_behaviour_set_->process();
      return;
    }
    it++;
  }
}

template <typename R, typename BS>
std::list<boost::shared_ptr<BS> > RobotInterface<R, BS>::getBehaviourSets() const
{
  return behaviour_sets_;
}

template <typename R, typename BS>
TaskPtr RobotInterface<R, BS>::getExecutingTask() const
{
  return active_behaviour_set_ ? active_behaviour_set_->getTask() : TaskPtr();
}

template <typename R, typename BS> bool RobotInterface<R, BS>::isIdle() const
{
  return !active_behaviour_set_;
}

template <typename R, typename BS>
void RobotInterface<R, BS>::addBehaviourSet(const BSPtr& behaviour_set)
{
  if (!behaviour_set)
  {
    ROS_ERROR_STREAM("The given robot's behaviour set must not be null.");
    return;
  }
  if (contains(*behaviour_set))
  {
    ROS_WARN_STREAM(*this << " robot already have the " << *behaviour_set
                          << " behaviour set.");
    return;
  }
  behaviour_sets_.push_back(behaviour_set);
}

template <typename R, typename BS>
bool RobotInterface<R, BS>::contains(const BS& behaviour_set) const
{
  typename std::list<BSPtr>::const_iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (**it == behaviour_set)
    {
      return true;
    }
    it++;
  }
  return false;
}
}

#endif // _ALLIANCE_ROBOT_INTEFACE_H_
