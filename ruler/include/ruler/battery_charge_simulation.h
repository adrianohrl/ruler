#ifndef _RULER_BATTERY_CHARGE_SIMULATION_H_
#define _RULER_BATTERY_CHARGE_SIMULATION_H_

#include <ros/rate.h>
#include "ruler/continuous_consumable_resource.h"
#include "ruler/task_simulation.h"

namespace ruler
{
class BatteryChargeSimulation : public utilities::Simulation
{
public:
  BatteryChargeSimulation(
      const std::string& robot_id,
      const utilities::ContinuousNoisySignalPtr& expected_sample_time,
      double slow_discharging_rate = 0.001, double low_threshold = 0.15,
      double critical_threshold = 0.05,
      const ros::Rate& low_warning_rate = ros::Rate(0.25),
      const ros::Rate& critical_warning_rate = ros::Rate(1.0));
  virtual ~BatteryChargeSimulation();
  virtual void update(const ros::Time& timestamp = ros::Time::now());
  void recharge();
  void stop(const ros::Time& timestamp = ros::Time::now());
  bool isRecharging(const ros::Time& timestamp = ros::Time::now()) const;
  bool isEmpty(const ros::Time& timestamp = ros::Time::now()) const;
  bool isLowLevel(const ros::Time& timestamp = ros::Time::now()) const;
  bool isCriticalLevel(const ros::Time& timestamp = ros::Time::now()) const;
  bool isFull(const ros::Time& timestamp = ros::Time::now()) const;
  double
  getRemainingCharge(const ros::Time& timestamp = ros::Time::now()) const;
  void setLowThreshold(double low_threshold);
  void setCriticalThreshold(double critical_threshold);
  void setLowWarningRate(const ros::Rate& low_warning_rate);
  void setCriticalWarningRate(const ros::Rate& critical_warning_rate);
  virtual std::string str() const;

private:
  ContinuousConsumableResourcePtr battery_charge_;
  TaskSimulationPtr recharge_;
  TaskSimulationPtr slowly_discharge_;
  bool recharging_;
  double low_threshold_;
  double critical_threshold_;
  ros::Rate low_warning_rate_;
  ros::Rate critical_warning_rate_;
};

typedef boost::shared_ptr<BatteryChargeSimulation> BatteryChargeSimulationPtr;
typedef boost::shared_ptr<BatteryChargeSimulation const>
    BatteryChargeSimulationConstPtr;
}

#endif // _RULER_BATTERY_CHARGE_SIMULATION_H_
