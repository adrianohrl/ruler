#ifndef _ALLIANCE_IMPATIENCE_RESET_H_
#define _ALLIANCE_IMPATIENCE_RESET_H_

#include <ros/time.h>

namespace alliance
{
class ImpatienceReset
{
public:
  ImpatienceReset(const ImpatienceReset& impatience_reset);
  virtual ~ImpatienceReset();
  bool reset(ros::Time timestamp = ros::Time::now()) const;

private:
};
}

#endif // _ALLIANCE_IMPATIENCE_RESET_H_
