#include "alliance_test/border_protection.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::BorderProtection, alliance::Layer)

namespace alliance_test
{
BorderProtection::BorderProtection() {}

BorderProtection::~BorderProtection() {}

void BorderProtection::process()
{
  Layer::process();
}
}
