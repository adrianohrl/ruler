#ifndef _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_
#define _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_

#include "utilities/beacon_signal_event.h"
#include <utilities/observer.h>

namespace utilities
{
class BeaconSignalObserver : public Observer
{
public:
  BeaconSignalObserver(std::string id);
  BeaconSignalObserver(const BeaconSignalObserver& observer);
  virtual ~BeaconSignalObserver();
  virtual void update(Event* event);
  virtual void update(BeaconSignalEvent* event) = 0;
};
}

#endif // _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_
