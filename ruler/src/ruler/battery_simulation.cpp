#include "ruler/battery_simulation.h"
#include "ruler/consumable_resource_reservation_request.h"
#include "utilities/functions/continuous_linear_function.h"

namespace ruler
{
BatterySimulation::BatterySimulation(
    const std::string& robot_id,
    const utilities::ContinuousNoisySignalPtr& expected_sample_time,
    double slow_discharging_rate, double low_threshold,
    double critical_threshold, const ros::Rate& low_warning_rate,
    const ros::Rate& critical_warning_rate)
    : Simulation::Simulation(),
      battery_charge_(new ContinuousConsumableResource(
          robot_id + "/battery_charge", robot_id + "'s Battery Charge", 1.0,
          1.0)),
      recharging_(false), low_threshold_(low_threshold),
      critical_threshold_(critical_threshold),
      low_warning_rate_(low_warning_rate),
      critical_warning_rate_(critical_warning_rate)
{
  if (slow_discharging_rate <= 0.0 || slow_discharging_rate >= 1.0)
  {
    throw utilities::Exception(
        "The battery charge slow (percentage) discharging rate must be "
        "within the (0.0; 1.0) interval.");
  }
  if (low_threshold_ <= 0.0 || low_threshold_ >= 1.0)
  {
    throw utilities::Exception(
        "The battery charge low (percentage) threshold must be "
        "within the (0.0; 1.0) interval.");
  }
  if (critical_threshold_ <= 0.0 || critical_threshold_ >= 1.0)
  {
    throw utilities::Exception(
        "The battery charge critical (percentage) threshold must be "
        "within the (0.0; 1.0) interval.");
  }
  if (low_threshold_ < critical_threshold_)
  {
    throw utilities::Exception("The battery charge low thresholh must be "
                               "greater then the critical one.");
  }
  ros::Time timestamp(ros::Time::now());
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp, ros::Duration(0.01)));
  ros::Time start_timestamp(expected_start->random());
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 / slow_discharging_rate),
      ros::Duration(1.0)));
  ros::Time end_timestamp(expected_end->random());
  TaskPtr slowly_discharge(
      new Task(battery_charge_->getId() + "/slow_discharging",
               battery_charge_->getId() + "/slow_discharging", expected_start,
               expected_end));
  slowly_discharge->addResourceReservationRequest(
      ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              slowly_discharge, battery_charge_,
              utilities::functions::ContinuousLinearFunctionPtr(
                  new utilities::functions::ContinuousLinearFunction(
                      start_timestamp - timestamp, end_timestamp - timestamp,
                      0.0, 1.0)))));
  slowly_discharge_.reset(
      new TaskSimulation(slowly_discharge, expected_sample_time));
}

BatterySimulation::~BatterySimulation() {}

void BatterySimulation::update(const ros::Time& timestamp)
{
  if (!recharging_)
  {
    slowly_discharge_->update(timestamp);
    if (isEmpty(timestamp))
    {
      ROS_FATAL("Battery charge is empty.");
    }
    else if (isCriticalLevel(timestamp))
    {
      ROS_ERROR_STREAM("Battery charge level is critical: "
                       << battery_charge_->getLevel(timestamp) * 100.0
                       << " [%]");
    }
    else if (isLowLevel(timestamp))
    {
      ROS_WARN_STREAM("Battery charge level is low: "
                      << battery_charge_->getLevel(timestamp) * 100.0
                      << " [%]");
    }
    return;
  }
  recharge_->update(timestamp);
  if (isFull(timestamp))
  {
    stop(timestamp);
    ROS_INFO("Battery charge is full.");
  }
}

void BatterySimulation::recharge() { recharging_ = true; }

void BatterySimulation::stop(const ros::Time& timestamp)
{
  recharging_ = false;
}

bool BatterySimulation::isRecharging(const ros::Time& timestamp) const
{
  return recharging_;
}

bool BatterySimulation::isEmpty(const ros::Time& timestamp) const
{
  return battery_charge_->getLevel(timestamp) == 0.0;
}

bool BatterySimulation::isLowLevel(const ros::Time& timestamp) const
{
  return battery_charge_->getLevel(timestamp) <= low_threshold_;
}

bool BatterySimulation::isCriticalLevel(const ros::Time& timestamp) const
{
  return battery_charge_->getLevel(timestamp) <= critical_threshold_;
}

bool BatterySimulation::isFull(const ros::Time& timestamp) const
{
  return battery_charge_->getLevel(timestamp) == 1.0;
}

double
BatterySimulation::getRemainingCharge(const ros::Time& timestamp) const
{
  return battery_charge_->getLevel(timestamp);
}

void BatterySimulation::setLowThreshold(double low_threshold)
{
  if (low_threshold > critical_threshold_)
  {
    low_threshold_ = low_threshold;
  }
}

void BatterySimulation::setCriticalThreshold(double critical_threshold)
{
  if (critical_threshold < low_threshold_)
  {
    critical_threshold_ = critical_threshold;
  }
}

std::string BatterySimulation::str() const
{
  std::stringstream ss;
  ss << "battery remaining charge: " << 100.0 * battery_charge_->getLevel()
     << " [%]";
  return ss.str();
}

void BatterySimulation::setLowWarningRate(
    const ros::Rate& low_warning_rate)
{
  low_warning_rate_ = low_warning_rate;
}

void BatterySimulation::setCriticalWarningRate(
    const ros::Rate& critical_warning_rate)
{
  critical_warning_rate_ = critical_warning_rate;
}
}
