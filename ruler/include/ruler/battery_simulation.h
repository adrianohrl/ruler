#ifndef _RULER_BATTERY_SIMULATION_H_
#define _RULER_BATTERY_SIMULATION_H_

#include <boost/enable_shared_from_this.hpp>
#include <ros/rate.h>
//#include "ruler/continuous_consumable_resource.h"
#include "ruler/consumable_resource.h"
#include "ruler/task_simulation.h"
#include "utilities/functions/unary_sample_holder.h"

namespace ruler
{
class BatterySimulation
    : public utilities::Simulation,
      public utilities::Subject,
      public boost::enable_shared_from_this<BatterySimulation>
{
public:
  BatterySimulation(
      const std::string& robot_id,
      const utilities::ContinuousNoisySignalPtr& expected_sample_time,
      double recharging_rate = 0.1, double slow_discharging_rate = 0.001,
      double low_threshold = 0.15, double critical_threshold = 0.05,
      const ros::Rate& low_warning_rate = ros::Rate(0.25),
      const ros::Rate& critical_warning_rate = ros::Rate(1.0),
      const ros::Duration& buffer_horizon = ros::Duration(10.0));
  virtual ~BatterySimulation();
  virtual void init();
  virtual void update(const ros::Time& timestamp = ros::Time::now());
  void recharge(const ros::Time& timestamp = ros::Time::now());
  void discharge(const std::string& task_id,
                 const utilities::NoisyDurationPtr& expected_duration,
                 const ros::Time& timestamp = ros::Time::now());
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
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  typedef std::list<TaskSimulationPtr>::iterator iterator;
  typedef std::list<TaskSimulationPtr>::const_iterator const_iterator;
  utilities::ContinuousNoisySignalPtr expected_sample_time_;
  ContinuousConsumableResourcePtr charge_;
  SampleHolderPtr recharging_;
  TaskSimulationPtr recharge_;
  TaskSimulationPtr slowly_discharge_;
  std::list<TaskSimulationPtr> discharges_;
  double recharging_rate_;
  double slow_discharging_rate_;
  double low_threshold_;
  double critical_threshold_;
  ros::Rate low_warning_rate_;
  ros::Time last_low_warning_timestamp_;
  ros::Rate critical_warning_rate_;
  ros::Time last_critical_warning_timestamp_;
};

typedef boost::shared_ptr<BatterySimulation> BatterySimulationPtr;
typedef boost::shared_ptr<BatterySimulation const> BatterySimulationConstPtr;
}

#endif // _RULER_BATTERY_SIMULATION_H_
