#ifndef _UTILITIES_BEACON_SIGNAL_SUBJECT_H_
#define _UTILITIES_BEACON_SIGNAL_SUBJECT_H_

#include <alliance_msgs/BeaconSignal.h>
#include "utilities/beacon_signal_event.h"
#include "utilities/beacon_signal_observer.h"
#include <utilities/subject.h>

namespace utilities
{
class BeaconSignalSubject : public Subject
{
public:
  BeaconSignalSubject(const std::string& id);
  BeaconSignalSubject(const BeaconSignalSubject& subject);
  virtual ~BeaconSignalSubject();

protected:
  void notify(const alliance_msgs::BeaconSignal& msg);
  void registerObserver(BeaconSignalObserver* observer);
};
}

#endif // _UTILITIES_BEACON_SIGNAL_SUBJECT_H_
