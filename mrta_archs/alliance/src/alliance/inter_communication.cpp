#include "alliance/inter_communication.h"

namespace alliance
{
InterCommunication::InterCommunication(const InterCommunication &inter_communication)
{

}

InterCommunication::~InterCommunication()
{

}

bool InterCommunication::received(ros::Time timestamp) const
{
  return false;
}
}
