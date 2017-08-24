#ifndef _ALLIANCE_IMPATIENCE_RESET_H_
#define _ALLIANCE_IMPATIENCE_RESET_H_

#include <ros/time.h>
#include <utilities/observer.h>

namespace alliance
{
class Robot;

class ImpatienceReset //: public utilities::Observer
{
public:
  ImpatienceReset(Robot* robot);
  ImpatienceReset(const ImpatienceReset& impatience_reset);
  virtual ~ImpatienceReset();
  bool reset(ros::Time timestamp = ros::Time::now()) const;
};
}

#endif // _ALLIANCE_IMPATIENCE_RESET_H_
