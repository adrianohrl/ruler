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
  BufferedFunction(std::string id, Function<T>* model,
                   ros::Duration interruption_delay,
                   ros::Duration buffer_horizon);
  BufferedFunction(const BufferedFunction<T>& function);
  virtual ~BufferedFunction();
  ros::Time getStartTimestamp() const;
  T getValue(ros::Time timestamp = ros::Time::now());
  virtual void update(Event* event);
  void update(ros::Time timestmap = ros::Time::now());
  void setInterruptionDelay(ros::Duration interruption_delay);
  void setBufferHorizon(ros::Duration buffer_horizon);

private:
  ros::Time start_timestamp_;
  ros::Time last_update_timestamp_;
  ros::Duration interruption_delay_;
  ros::Duration buffer_horizon_;
  Function<T>* model_;
  std::list<Function<T>*> functions_;
  T getValue(double d) const;
  void cleanBuffer(double d);

};

template <typename T>
BufferedFunction<T>::BufferedFunction(std::string id, Function<T>* model,
                                      ros::Duration interruption_delay,
                                      ros::Duration buffer_horizon)
    : Observer::Observer(id), model_(model), start_timestamp_(ros::Time::now()),
      interruption_delay_(interruption_delay), buffer_horizon_(buffer_horizon),
      last_update_timestamp_(start_timestamp_)
{
}

template <typename T>
BufferedFunction<T>::BufferedFunction(const BufferedFunction<T>& function)
    : Observer::Observer(function), model_(function.model_),
      start_timestamp_(ros::Time::now()),
      interruption_delay_(function.interruption_delay_),
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

template <typename T> T BufferedFunction<T>::getValue(ros::Time timestamp)
{
  double d((timestamp - start_timestamp_).toSec());
  cleanBuffer(d);
  return getValue(d);
}

template <typename T> void BufferedFunction<T>::update(Event* event)
{
  update(event->getTimestamp());
}

template <typename T> void BufferedFunction<T>::update(ros::Time timestamp)
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
    function = model_->clone();
    function->setD0(d);
    function->setDf(interruption_delay_ + d);
    functions_.push_back(function);
    return;
  }
  function = functions_.back();
  if (function->getDf() < d.toSec())
  {
    function = model_->clone();
    function->setD0(d);
    function->setDf(interruption_delay_ + d);
    functions_.push_back(function);
  }
  else
  {
    function->setDf(interruption_delay_ + d);
  }
}

template <typename T>
void BufferedFunction<T>::setInterruptionDelay(ros::Duration interruption_delay)
{
  interruption_delay_ = interruption_delay;
}

template <typename T>
void BufferedFunction<T>::setBufferHorizon(ros::Duration buffer_horizon)
{
  buffer_horizon_ = buffer_horizon;
}

template <typename T>
void BufferedFunction<T>::cleanBuffer(double d)
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
  /*
  functions_.erase(functions_.begin());
  if (!functions_.empty())
  {
    Function<T>* function = functions_.front();
    d = ros::Duration(function->getDf()) - d + buffer_horizon_;
    ROS_WARN_STREAM("resetting clean buffer trigger to "
                    << d.toSec() << " [s] after start.");
    return;
  }
  start_timestamp_ = ros::Time::now();
  last_update_timestamp_ = start_timestamp_;*/
}
}
}

#endif // _UTILITIES_BUFFERED_FUNCTION_H_
