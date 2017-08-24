#include "utilities/beacon_signal_observer.h"

namespace utilities
{
BeaconSignalObserver::BeaconSignalObserver(std::string id)
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
  throw utilities::Exception("This update method must not be used.");
}
}
