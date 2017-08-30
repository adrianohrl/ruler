#ifndef _UTILITIES_BUFFERED_FUNCTION_H_
#define _UTILITIES_BUFFERED_FUNCTION_H_

#include <list>
#include <ros/node_handle.h>
#include "utilities/observer.h"
#include "utilities/functions/pulse_function.h"

namespace utilities
{
namespace functions
{
template <typename T> class BufferedFunction : public Observer
{
public:
  BufferedFunction(const std::string& id, Function<T>* model,
                   const ros::Duration& timeout_duration,
                   const ros::Duration& buffer_horizon,
                   const ros::Time& start_timestamp = ros::Time::now());
  BufferedFunction(const BufferedFunction<T>& function);
  virtual ~BufferedFunction();
  ros::Time getStartTimestamp() const;
  ros::Duration getTimeoutDuration() const;
  ros::Duration getBufferHorizon() const;
  T getValue(const ros::Time& timestamp = ros::Time::now());
  virtual void update(Event* event);
  void update(Event* event, Function<T>* model);
  virtual void update(const ros::Time& timestamp);
  void update(const ros::Time& timestamp, Function<T>* model);
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  void setBufferHorizon(const ros::Duration& buffer_horizon);

protected:
  T getValue(double d) const;

private:
  ros::Time start_timestamp_;
  ros::Time last_update_timestamp_;
  ros::Duration timeout_duration_;
  ros::Duration buffer_horizon_;
  Function<T>* model_;
  std::list<Function<T>*> functions_;
  void cleanBuffer(double d);
};

template <typename T>
BufferedFunction<T>::BufferedFunction(const std::string& id, Function<T>* model,
                                      const ros::Duration& timeout_duration,
                                      const ros::Duration& buffer_horizon,
                                      const ros::Time& start_timestamp)
    : Observer::Observer(id), model_(model), start_timestamp_(start_timestamp),
      timeout_duration_(timeout_duration), buffer_horizon_(buffer_horizon),
      last_update_timestamp_(start_timestamp_)
{
}

template <typename T>
BufferedFunction<T>::BufferedFunction(const BufferedFunction<T>& function)
    : Observer::Observer(function), model_(function.model_),
      start_timestamp_(ros::Time::now()),
      timeout_duration_(function.timeout_duration_),
      buffer_horizon_(function.buffer_horizon_),
      last_update_timestamp_(start_timestamp_)
{
}

template <typename T> BufferedFunction<T>::~BufferedFunction()
{
  typename std::list<Function<T>*>::iterator it(functions_.begin());
  while (it != functions_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
  if (model_)
  {
    delete model_;
    model_ = NULL;
  }
}

template <typename T> ros::Time BufferedFunction<T>::getStartTimestamp() const
{
  return start_timestamp_;
}

template <typename T>
ros::Duration BufferedFunction<T>::getTimeoutDuration() const
{
  return timeout_duration_;
}

template <typename T>
ros::Duration BufferedFunction<T>::getBufferHorizon() const
{
  return buffer_horizon_;
}

template <typename T> T BufferedFunction<T>::getValue(double d) const
{
  double q(0.0);
  typename std::list<Function<T>*>::const_iterator it(functions_.begin());
  while (it != functions_.end())
  {
    Function<T>* function = *it;
    if (function->getDf() >= d)
    {
      if (function->isNegated())
      {
        q -= function->getValue(d);
      }
      else
      {
        q += function->getValue(d);
      }
    }
    it++;
  }
  return q;
}

template <typename T>
T BufferedFunction<T>::getValue(const ros::Time& timestamp)
{
  double d((timestamp - start_timestamp_).toSec());
  cleanBuffer(d);
  return getValue(d);
}

template <typename T> void BufferedFunction<T>::update(Event* event)
{
  update(event->getTimestamp(), model_);
}

template <typename T>
void BufferedFunction<T>::update(Event* event, Function<T>* model)
{
  update(event->getTimestamp(), model);
  if (model_)
  {
    delete model_;
  }
  model_ = model;
}

template <typename T>
void BufferedFunction<T>::update(const ros::Time& timestamp)
{
  update(timestamp, model_);
}

template <typename T>
void BufferedFunction<T>::update(const ros::Time& timestamp, Function<T>* model)
{
  if (timestamp <= last_update_timestamp_)
  {
    ROS_WARN_STREAM("Ignoring already past event in " << *this << ".");
    return;
  }
  last_update_timestamp_ = timestamp;
  ros::Duration d(last_update_timestamp_ - start_timestamp_);
  Function<T>* function;
  if (functions_.empty())
  {
    function = model->clone();
    function->setD0(d);
    function->setDf(timeout_duration_ + d);
    functions_.push_back(function);
    return;
  }
  function = functions_.back();
  if (function->getDf() < d.toSec())
  {
    function = model->clone();
    function->setD0(d);
    function->setDf(timeout_duration_ + d);
    functions_.push_back(function);
  }
  else
  {
    function->setDf(timeout_duration_ + d);
  }
}

template <typename T>
void BufferedFunction<T>::setTimeoutDuration(
    const ros::Duration& timeout_duration)
{
  timeout_duration_ = timeout_duration;
}

template <typename T>
void BufferedFunction<T>::setBufferHorizon(const ros::Duration& buffer_horizon)
{
  buffer_horizon_ = buffer_horizon;
}

template <typename T> void BufferedFunction<T>::cleanBuffer(double d)
{
  if (functions_.empty())
  {
    return;
  }
  d -= buffer_horizon_.toSec();
  typename std::list<Function<T>*>::iterator it(functions_.begin());
  while (it != functions_.end())
  {
    Function<T>* function = *it;
    if (function->getDf() > d)
    {
      return;
    }
    delete function;
    it = functions_.erase(it);
  }
}
}
}

#endif // _UTILITIES_BUFFERED_FUNCTION_H_
