#ifndef _ALLIANCE_INTER_COMMUNICATION_H_
#define _ALLIANCE_INTER_COMMUNICATION_H_

#include <ros/time.h>

namespace alliance
{
class InterCommunication
{
public:
  InterCommunication(const InterCommunication& inter_communication);
  virtual ~InterCommunication();
  bool received(ros::Time timestamp = ros::Time::now()) const;

private:
};
}

#endif // _ALLIANCE_INTER_COMMUNICATION_H_
