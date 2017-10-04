#ifndef _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
#define _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_

#include "alliance/behaviour_set_interface.h"
#include "alliance/layer.h"
#include <list>
#include <pluginlib/class_loader.h>
#include "utilities/beacon_signal_observer.h"

namespace alliance
{
class BehavedRobot;
typedef boost::shared_ptr<BehavedRobot> BehavedRobotPtr;
typedef boost::shared_ptr<BehavedRobot const> BehavedRobotConstPtr;

class LayeredBehaviourSet
    : public BehaviourSetInterface<BehavedRobot>,
      public utilities::BeaconSignalObserver
{
public:
  LayeredBehaviourSet(const BehavedRobotPtr& robot, const TaskPtr& task,
                      const ros::Duration& buffer_horizon,
                      const ros::Duration& timeout_duration);
  virtual ~LayeredBehaviourSet();
  virtual void process();
  void addLayer(const std::string& layer_name);
  void addLayer(const LayerPtr& layer);
  virtual void update(const utilities::BeaconSignalEventConstPtr& event);

private:
  typedef std::list<LayerPtr>::iterator layers_iterator;
  typedef std::list<LayerPtr>::const_iterator layers_const_iterator;
  BehavedRobotPtr robot_;
  pluginlib::ClassLoader<Layer> loader_;
  std::list<LayerPtr> layers_;
  bool contains(const std::string& layer_name) const;
};

typedef boost::shared_ptr<LayeredBehaviourSet> LayeredBehaviourSetPtr;
typedef boost::shared_ptr<LayeredBehaviourSet const>
    LayeredBehaviourSetConstPtr;
}

#endif // _ALLIANCE_LAYERED_BEHAVIOUR_SET_H_
