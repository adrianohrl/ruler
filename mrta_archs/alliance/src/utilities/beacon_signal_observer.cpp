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

void BeaconSignalObserver::update(Event* event)
{
  if (typeid(*event) == typeid(BeaconSignalEvent))
  {
    update((BeaconSignalEvent*)event);
  }
}
}
