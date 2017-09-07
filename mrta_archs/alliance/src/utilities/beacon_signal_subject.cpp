#include "utilities/beacon_signal_subject.h"

namespace utilities
{

BeaconSignalSubject::BeaconSignalSubject(const std::string& id)
    : Subject::Subject(id)
{
}

BeaconSignalSubject::BeaconSignalSubject(const BeaconSignalSubject& subject)
    : Subject::Subject(subject)
{
}

BeaconSignalSubject::~BeaconSignalSubject() {}

void BeaconSignalSubject::registerObserver(
    const BeaconSignalObserverPtr& observer)
{
  Subject::registerObserver(observer);
}

void BeaconSignalSubject::notify(const alliance_msgs::BeaconSignal& msg)
{
  BeaconSignalEventPtr event(new BeaconSignalEvent(shared_from_this(), msg));
  Subject::notify(event);
}
}
