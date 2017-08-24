#include "alliance/impatience_reset.h"
#include "alliance/robot.h"

namespace alliance
{
ImpatienceReset::ImpatienceReset(Robot *robot)
  //: Observer::Observer(robot->getId() + "/impatience_reset")
{

}

ImpatienceReset::ImpatienceReset(const ImpatienceReset &impatience_reset)
  //: Observer::Observer(impatience_reset)
{

}

ImpatienceReset::~ImpatienceReset()
{

}

bool ImpatienceReset::reset(ros::Time timestamp) const
{
  return false;
}
}
