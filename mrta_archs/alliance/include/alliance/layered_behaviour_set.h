#ifndef _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
#define _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/layer.h"
#include <list>
#include <pluginlib/class_loader.h>

namespace alliance
{
class BehavedRobot;

class LayeredBehaviourSet : public BehaviourSetInterface
{
public:
  LayeredBehaviourSet(BehavedRobot* robot, Task* task);
  virtual ~LayeredBehaviourSet();
  virtual void process();
  void addLayer(const std::string& layer_name);

private:
  pluginlib::ClassLoader<Layer> loader_;
  std::list<boost::shared_ptr<Layer> > layers_;
  bool contains(const std::string &layer_name) const;
};
}

#endif // _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
