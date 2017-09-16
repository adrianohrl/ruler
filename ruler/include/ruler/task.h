/**
 *  This header file defines the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_H_
#define _RULER_TASK_H_

#include <boost/enable_shared_from_this.hpp>
#include <geometry_msgs/Pose.h>
#include "ruler/resource_reservation_request.h"
#include "ruler/resource_interface.h"
#include "ruler/task_event.h"
#include <ruler_msgs/Task.h>
#include "utilities/noisy_duration.h"
#include "utilities/noisy_time.h"
#include "utilities/ros_message_converter.h"
#include "utilities/subject.h"

namespace ruler
{
template <typename T> class Resource;

class Task : public utilities::Subject,
             public utilities::ROSMessageConverter<ruler_msgs::Task>,
             public boost::enable_shared_from_this<Task>
{
public:
  Task(const std::string& id, const std::string& name,
       const utilities::NoisyTimePtr& start,
       const utilities::NoisyTimePtr& end,
       bool preemptive = false,
       const std::list<geometry_msgs::Pose>& waypoints =
           std::list<geometry_msgs::Pose>());
  Task(const ruler_msgs::Task& msg);
  Task(const Task& task);
  virtual ~Task();
  void addResourceReservationRequest(const ResourceReservationRequestPtr &request);
  void addResource(const ResourceInterfacePtr& resource);
  void removeResource(const ResourceInterfacePtr& resource);
  void start(const ros::Time& timestamp = ros::Time::now());
  void interrupt(const ros::Time& timestamp = ros::Time::now());
  void resume(const ros::Time& timestamp = ros::Time::now());
  void finish(const ros::Time& timestamp = ros::Time::now());
  void clearResources();
  double getDuration(const ros::Time& timestamp = ros::Time::now()) const;
  std::string getName() const;
  bool isPreemptive() const;
  ros::Time getStartTimestamp() const;
  ros::Time getLastInterruptionTimestamp() const;
  ros::Time getLastResumeTimestamp() const;
  ros::Time getEndTimestamp() const;
  bool hasStarted() const;
  bool isInterrupted() const;
  bool isRunning() const;
  bool hasFinished() const;
  utilities::NoisyTimePtr getExpectedStart() const;
  utilities::NoisyTimePtr getExpectedEnd() const;
  utilities::NoisyDurationPtr getExpectedDuration() const;
  std::list<geometry_msgs::Pose> getWaypoints() const;
  double getDistance() const;
  virtual ruler_msgs::Task toMsg() const;
  using Subject::operator==;
  virtual bool operator==(const ruler_msgs::Task& msg) const;
  using Subject::operator!=;

protected:
  typedef utilities::Interval<ros::Time>::Ptr TimeIntervalPtr;

private:
  std::string name_;
  bool preemptive_;
  utilities::NoisyTimePtr expected_start_;
  utilities::NoisyTimePtr expected_end_;
  utilities::NoisyDurationPtr expected_duration_;
  ros::Time start_timestamp_;
  ros::Time last_interruption_timestamp_;
  ros::Time end_timestamp_;
  ros::Time last_event_timestamp_;
  std::list<TimeIntervalPtr> interruption_intervals_;
  std::list<ResourceReservationRequestPtr> resource_reservation_requests_;
  std::list<geometry_msgs::Pose> waypoints_;
};

typedef boost::shared_ptr<Task> TaskPtr;
typedef boost::shared_ptr<Task const> TaskConstPtr;
}

#endif // _RULER_TASK_H_
