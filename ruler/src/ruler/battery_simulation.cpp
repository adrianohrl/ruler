#include "ruler/battery_simulation.h"
#include "ruler/consumable_resource_reservation_request.h"
#include "utilities/functions/continuous_linear_function.h"
#include "utilities/toggle_event.h"

namespace ruler
{
BatterySimulation::BatterySimulation(
    const std::string& robot_id,
    const utilities::ContinuousNoisySignalPtr& expected_sample_time,
    double recharging_rate, double slow_discharging_rate, double low_threshold,
    double critical_threshold, const ros::Rate& low_warning_rate,
    const ros::Rate& critical_warning_rate, const ros::Duration& buffer_horizon)
    : Simulation::Simulation(), Subject::Subject(robot_id + "/battery_charge"),
      charge_(new ContinuousConsumableResource(robot_id + "/battery_charge",
                                               robot_id + "'s Battery Charge",
                                               1.0, 1.0)),
      low_threshold_(low_threshold),
      recharging_(
          new SampleHolder(robot_id + "/battery/recharging", buffer_horizon)),
      recharging_rate_(recharging_rate),
      slow_discharging_rate_(slow_discharging_rate),
      critical_threshold_(critical_threshold),
      low_warning_rate_(low_warning_rate),
      critical_warning_rate_(critical_warning_rate),
      expected_sample_time_(expected_sample_time)
{
  if (recharging_rate <= 0.0 || recharging_rate >= 1.0)
  {
    throw utilities::Exception(
        "The battery charge (percentage) recharging rate must be "
        "within the (0.0; 1.0) interval.");
  }
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
}

BatterySimulation::~BatterySimulation() {}

void BatterySimulation::init() { stop(); }

void BatterySimulation::update(const ros::Time& timestamp)
{
  if (!isEmpty(timestamp))
  {
    for (iterator it(discharges_.begin());
         it != discharges_.end() && !isEmpty(timestamp); it++)
    {
      TaskSimulationPtr discharge(*it);
      if (discharge->getTask()->hasFinished())
      {
        it = discharges_.erase(it);
        it--;
        continue;
      }
      discharge->update(timestamp);
    }
    if (!isRecharging(timestamp))
    {
      slowly_discharge_->update(timestamp);
      if (isEmpty(timestamp))
      {
        ROS_FATAL("Battery charge is empty.");
        slowly_discharge_->abort(timestamp);
        return;
      }
      if (isCriticalLevel(timestamp))
      {
        if (timestamp - last_critical_warning_timestamp_ >=
            critical_warning_rate_.expectedCycleTime())
        {
          ROS_ERROR_STREAM("Battery charge level is critical: "
                           << charge_->getLevel(timestamp) * 100.0 << " [%]");
          last_critical_warning_timestamp_ = timestamp;
        }
      }
      else if (isLowLevel(timestamp) && timestamp - last_low_warning_timestamp_ >=
                   low_warning_rate_.expectedCycleTime())
      {
        ROS_WARN_STREAM("Battery charge level is low: "
                        << charge_->getLevel(timestamp) * 100.0 << " [%]");
        last_low_warning_timestamp_ = timestamp;
      }
      return;
    }
  }
  if (isRecharging(timestamp))
  {
    if (!recharge_)
    {
      ROS_FATAL_STREAM("Unable to recharge " << getId()
                                             << ": past event timestamp!!!");
    }
    recharge_->update(timestamp);
    if (isFull(timestamp))
    {
      stop(timestamp);
      ROS_INFO("Battery charge is full.");
    }
  }
}

void BatterySimulation::recharge(const ros::Time& timestamp)
{
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp, ros::Duration(1.0)));
  ros::Time start_timestamp(expected_start->random());
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration((1.0 - charge_->getLevel(timestamp)) /
                                recharging_rate_),
      ros::Duration(2.0)));
  ros::Time end_timestamp(expected_end->random());
  TaskPtr recharge(new Task(charge_->getId() + "/recharging",
                            charge_->getId() + "/recharging", expected_start,
                            expected_end));
  recharge->addResourceReservationRequest(
      ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              recharge, charge_,
              utilities::functions::ContinuousLinearFunctionPtr(
                  new utilities::functions::ContinuousLinearFunction(
                      start_timestamp - timestamp, end_timestamp - timestamp,
                      0.0, 1.0)),
              false)));
  recharge_.reset(new TaskSimulation(recharge, expected_sample_time_));
  utilities::ToggleEventConstPtr event(
      new utilities::ToggleEvent(shared_from_this(), true, timestamp));
  recharging_->update(event);
}

void BatterySimulation::discharge(
    const std::string& task_id,
    const utilities::NoisyDurationPtr& expected_duration,
    const ros::Time& timestamp)
{
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp, ros::Duration(0.5)));
  ros::Time start_timestamp(expected_start->random());
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + expected_duration->random(),
      ros::Duration(expected_duration->getStandardDeviation())));
  ros::Time end_timestamp(expected_end->random());
  TaskPtr discharge(new Task(charge_->getId() + "/discharging/" + task_id,
                             charge_->getId() + "/discharging/" + task_id,
                             expected_start, expected_end));
  discharge->addResourceReservationRequest(
      ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              discharge, charge_,
              utilities::functions::ContinuousLinearFunctionPtr(
                  new utilities::functions::ContinuousLinearFunction(
                      start_timestamp - timestamp, end_timestamp - timestamp,
                      0.0, 1.0)))));
  TaskSimulationPtr discharge_ptr(
      new TaskSimulation(discharge, expected_sample_time_));
  discharges_.push_back(discharge_ptr);
}

void BatterySimulation::stop(const ros::Time& timestamp)
{
  utilities::NoisyTimePtr expected_start(
      new utilities::NoisyTime(timestamp, ros::Duration(0.01)));
  ros::Time start_timestamp(expected_start->random());
  utilities::NoisyTimePtr expected_end(new utilities::NoisyTime(
      timestamp + ros::Duration(1.0 / slow_discharging_rate_),
      ros::Duration(1.0)));
  ros::Time end_timestamp(expected_end->random());
  TaskPtr slowly_discharge(new Task(charge_->getId() + "/slow_discharging",
                                    charge_->getId() + "/slow_discharging",
                                    expected_start, expected_end));
  slowly_discharge->addResourceReservationRequest(
      ContinuousConsumableResourceReservationRequestPtr(
          new ruler::ContinuousConsumableResourceReservationRequest(
              slowly_discharge, charge_,
              utilities::functions::ContinuousLinearFunctionPtr(
                  new utilities::functions::ContinuousLinearFunction(
                      start_timestamp - timestamp, end_timestamp - timestamp,
                      0.0, 1.0)))));
  slowly_discharge_.reset(
      new TaskSimulation(slowly_discharge, expected_sample_time_));
  recharge_.reset();
  utilities::ToggleEventConstPtr event(
      new utilities::ToggleEvent(shared_from_this(), false, timestamp));
  recharging_->update(event);
}

bool BatterySimulation::isRecharging(const ros::Time& timestamp) const
{
  return recharging_->getValue(timestamp);
}

bool BatterySimulation::isEmpty(const ros::Time& timestamp) const
{
  return charge_->getLevel(timestamp) < 0.01;
}

bool BatterySimulation::isLowLevel(const ros::Time& timestamp) const
{
  return charge_->getLevel(timestamp) <= low_threshold_;
}

bool BatterySimulation::isCriticalLevel(const ros::Time& timestamp) const
{
  return charge_->getLevel(timestamp) <= critical_threshold_;
}

bool BatterySimulation::isFull(const ros::Time& timestamp) const
{
  return charge_->getLevel(timestamp) == 1.0;
}

double BatterySimulation::getRemainingCharge(const ros::Time& timestamp) const
{
  return charge_->getLevel(timestamp);
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
  ss << "battery remaining charge: " << 100.0 * charge_->getLevel() << " [%]";
  return ss.str();
}

void BatterySimulation::setLowWarningRate(const ros::Rate& low_warning_rate)
{
  low_warning_rate_ = low_warning_rate;
}

void BatterySimulation::setCriticalWarningRate(
    const ros::Rate& critical_warning_rate)
{
  critical_warning_rate_ = critical_warning_rate;
}
}
