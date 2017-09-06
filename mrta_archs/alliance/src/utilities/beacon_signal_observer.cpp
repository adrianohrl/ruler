#include "utilities/beacon_signal_observer.h"

namespace utilities
{
BeaconSignalObserver::BeaconSignalObserver(const std::string& id)
    : Observer::Observer(id)
{
}

BeaconSignalObserver::BeaconSignalObserver(const BeaconSignalObserver& observer)
    : Observer::Observer(observer)
{
}

BeaconSignalObserver::~BeaconSignalObserver() {}

void BeaconSignalObserver::update(const EventConstPtr& event)
{
  BeaconSignalEventConstPtr beacon_signal_event(
      boost::dynamic_pointer_cast<BeaconSignalEvent const>(event));
  if (beacon_signal_event)
  {
    update(beacon_signal_event);
  }
}
}
