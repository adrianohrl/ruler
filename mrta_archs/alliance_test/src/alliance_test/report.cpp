#include "alliance_test/report.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::Report, alliance::Layer)

namespace alliance_test
{
Report::Report() {}

Report::~Report() {}

void Report::process() { Layer::process(); }
}
