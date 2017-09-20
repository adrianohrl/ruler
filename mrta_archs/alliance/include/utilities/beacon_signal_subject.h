#ifndef _UTILITIES_BEACON_SIGNAL_SUBJECT_H_
#define _UTILITIES_BEACON_SIGNAL_SUBJECT_H_

#include <alliance_msgs/BeaconSignal.h>
#include "utilities/beacon_signal_event.h"
#include "utilities/beacon_signal_observer.h"
#include <utilities/subject.h>

namespace utilities
{
class BeaconSignalSubject
    : public Subject
{
public:
  BeaconSignalSubject(const std::string& id);
  BeaconSignalSubject(const BeaconSignalSubject& subject);
  virtual ~BeaconSignalSubject();
  void notify(const BeaconSignalEventConstPtr &event);
  void registerObserver(const BeaconSignalObserverPtr& observer);
};

typedef boost::shared_ptr<BeaconSignalSubject> BeaconSignalSubjectPtr;
typedef boost::shared_ptr<BeaconSignalSubject const>
    BeaconSignalSubjectConstPtr;
}

#endif // _UTILITIES_BEACON_SIGNAL_SUBJECT_H_
