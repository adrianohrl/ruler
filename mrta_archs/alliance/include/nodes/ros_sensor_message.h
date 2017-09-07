#ifndef _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_
#define _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_

#include "alliance/sensor.h"
#include <alliance_msgs/SensorFeedback.h>
#include <ros/node_handle.h>
#include <utilities/functions/unary_sample_holder.h>

namespace nodes
{
template <typename M> class ROSSensorMessage : public alliance::Sensor
{
public:
  typedef boost::shared_ptr<ROSSensorMessage<M> > Ptr;
  ROSSensorMessage(const std::string& id, const ros::NodeHandlePtr& nh,
                   const std::string& topic_name,
                   const ros::Duration& timeout_duration);
  virtual ~ROSSensorMessage();
  void publish();
  M getMsg() const;

private:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  SampleHolderPtr applicable_;
  M msg_;
  ros::NodeHandlePtr nh_;
  ros::Publisher feedback_pub_;
  ros::Subscriber sensor_sub_;
  void sensorCallback(const M& msg);
};

template <typename M>
ROSSensorMessage<M>::ROSSensorMessage(const std::string& id,
                                      const ros::NodeHandlePtr& nh,
                                      const std::string& topic_name,
                                      const ros::Duration& timeout_duration)
    : Sensor::Sensor(id), nh_(nh),
      applicable_(new SampleHolder(
          topic_name, timeout_duration,
          ros::Duration(10 * timeout_duration.toSec())))
{

  feedback_pub_ = nh_->advertise<alliance_msgs::SensorFeedback>(
      "alliance/sensory_feedback", 10);
  sensor_sub_ = nh_->subscribe(topic_name, 10,
                               &ROSSensorMessage<M>::sensorCallback, this);
}

template <typename M> ROSSensorMessage<M>::~ROSSensorMessage()
{
  feedback_pub_.shutdown();
  sensor_sub_.shutdown();
}

template <typename M> void ROSSensorMessage<M>::publish()
{
  alliance_msgs::SensorFeedback msg;
  msg.header.stamp = ros::Time::now();
  msg.applicable = applicable_->getValue();
  feedback_pub_.publish(msg);
}

template <typename M> M ROSSensorMessage<M>::getMsg() const { return msg_; }

template <typename M> void ROSSensorMessage<M>::sensorCallback(const M& msg)
{
  applicable_->update(ros::Time::now());
  msg_ = msg;
}
}

#endif // _ALLIANCE_NODE_ROS_SENSOR_MESSAGE_H_
