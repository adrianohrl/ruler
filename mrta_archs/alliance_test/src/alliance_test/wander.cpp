#include "alliance_test/wander.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::Wander, alliance::Layer)

namespace alliance_test
{
Wander::Wander() {}

Wander::~Wander() {}

void Wander::process() { Layer::process(); }
}
