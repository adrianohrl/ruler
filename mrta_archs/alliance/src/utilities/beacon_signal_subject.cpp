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

BeaconSignalSubjectPtr BeaconSignalSubject::shared_from_this()
{
  return boost::dynamic_pointer_cast<BeaconSignalSubject>(
      Subject::shared_from_this());
}

void BeaconSignalSubject::notify(const alliance_msgs::BeaconSignal& msg)
{
  BeaconSignalEventPtr event(new BeaconSignalEvent(shared_from_this(), msg));
  Subject::notify(event);
}
}
