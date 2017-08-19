#include <pluginlib/class_list_macros.h>
#include "ruler/metrics_plugins.h"

PLUGINLIB_EXPORT_CLASS(ruler::BatteryConsumptionEstimator, ruler::MetricsEstimator)
PLUGINLIB_EXPORT_CLASS(ruler::DisplacementEstimator, ruler::MetricsEstimator)

