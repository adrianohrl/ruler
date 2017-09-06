#ifndef _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_
#define _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_

#include "utilities/beacon_signal_event.h"
#include <utilities/observer.h>

namespace utilities
{
class BeaconSignalObserver : public Observer
{
public:
  BeaconSignalObserver(const std::string& id);
  BeaconSignalObserver(const BeaconSignalObserver& observer);
  virtual ~BeaconSignalObserver();
  virtual void update(const EventConstPtr& event);
  virtual void update(const BeaconSignalEventConstPtr& event) = 0;
};

typedef boost::shared_ptr<BeaconSignalObserver> BeaconSignalObserverPtr;
typedef boost::shared_ptr<BeaconSignalObserver const> BeaconSignalObserverConstPtr;
}

#endif // _UTILLITIES_BEACON_SIGNAL_OBSERVER_H_
