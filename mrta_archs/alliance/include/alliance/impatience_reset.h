#ifndef _ALLIANCE_IMPATIENCE_RESET_H_
#define _ALLIANCE_IMPATIENCE_RESET_H_

#include "alliance/inter_communication.h"
#include <ros/time.h>
#include <utilities/observer.h>

namespace alliance
{
class ImpatienceReset
{
public:
  ImpatienceReset(InterCommunication* monitor);
  ImpatienceReset(const ImpatienceReset& impatience_reset);
  virtual ~ImpatienceReset();
  bool isResetted(const ros::Time& timestamp = ros::Time::now()) const;

private:
  InterCommunication* monitor_;
};
}

#endif // _ALLIANCE_IMPATIENCE_RESET_H_
