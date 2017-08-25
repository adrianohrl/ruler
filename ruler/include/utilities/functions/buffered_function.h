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
  T getValue(double d) const;
  T getValue(ros::Duration d) const;
  T getValue(ros::Time timestamp = ros::Time::now()) const;
  virtual void update(Event* event);
  void update(ros::Time timestmap = ros::Time::now());
  void setInterruptionDelay(ros::Duration interruption_delay);
  void setBufferHorizon(ros::Duration buffer_horizon);

private:
  ros::Time start_timestamp_;
  ros::Time last_update_timestamp_;
  ros::Duration interruption_delay_;
  ros::Duration buffer_horizon_;
  ros::Timer clean_buffer_trigger_;
  Function<T>* model_;
  Function<T>* interruption_model_;
  std::list<Function<T>*> functions_;
  std::list<Function<T>*> interruption_functions_;
  void setUp();
  void cleanBuffer(const ros::TimerEvent& event);
};

template <typename T>
BufferedFunction<T>::BufferedFunction(std::string id, Function<T>* model,
                                      ros::Duration interruption_delay,
                                      ros::Duration buffer_horizon)
    : Observer::Observer(id), model_(model),
      interruption_model_(model->clone()), start_timestamp_(ros::Time::now()),
      interruption_delay_(interruption_delay), buffer_horizon_(buffer_horizon),
      last_update_timestamp_(start_timestamp_)
{
  interruption_model_->setNegated(!model_->getName());
  setUp();
}

template <typename T>
BufferedFunction<T>::BufferedFunction(const BufferedFunction<T>& function)
    : Observer::Observer(function), model_(function.model_),
      start_timestamp_(ros::Time::now()),
      interruption_delay_(function.interruption_delay_),
      buffer_horizon_(function.buffer_horizon_),
      last_update_timestamp_(start_timestamp_)
{
  setUp();
}

template <typename T> BufferedFunction<T>::~BufferedFunction()
{
  clean_buffer_trigger_.stop();
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
  it = interruption_functions_.begin();
  while (it != interruption_functions_.end())
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
  if (interruption_model_)
  {
    delete interruption_model_;
    interruption_model_ = NULL;
  }
}

template <typename T> void BufferedFunction<T>::setUp()
{
  ros::NodeHandle nh;
  clean_buffer_trigger_ = nh.createTimer(
      buffer_horizon_, &BufferedFunction<T>::cleanBuffer, this, true, false);
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
    if (function->isNegated())
    {
      q -= function->getValue(d);
    }
    else
    {
      q += function->getValue(d);
    }
    it++;
  }
  it = interruption_functions_.begin();
  while (it != interruption_functions_.end())
  {
    Function<T>* function = *it;
    if (function->isNegated())
    {
      q -= function->getValue(d);
    }
    else
    {
      q += function->getValue(d);
    }
    it++;
  }
  return q;
}

template <typename T> T BufferedFunction<T>::getValue(ros::Duration d) const
{
  return getValue(d.toSec());
}

template <typename T> T BufferedFunction<T>::getValue(ros::Time timestamp) const
{
  return getValue((timestamp - start_timestamp_).toSec());
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
  Function<T>* interruption_function;
  if (functions_.empty())
  {
    function = model_->clone();
    interruption_function = interruption_model_->clone();
    // terminar aki

    // acho que e melhor nao existir a interruption_model_
    // mas depois de ter configurado a function
    // eu clono ela e nego ela

    // pq tudo que eu faco com a function,
    // eu vou ter que com a sua interruption_function

    // o problema eh que o d0 delas eh diferente,
    // so a dinamica das duas que tem q ser a mesma

    function->setD0(d);
    function->setDf(interruption_delay_ + d);
    functions_.push_back(function);
    interruption_functions_.push_back(interruption_function);
    clean_buffer_trigger_.setPeriod(interruption_delay_ + buffer_horizon_);
    return;
  }
  function = functions_.back();
  if (function->getDf() < d.toSec())
  {
    function = model_->clone();
    interruption_function = interruption_model_->clone();
    // terminar aki
    function->setD0(d);
    function->setDf(interruption_delay_ + d);
    functions_.push_back(function);
    interruption_functions_.push_back(interruption_function);
  }
  else
  {
    interruption_function = interruption_functions_.back();
    // terminar aki
    function->setDf(interruption_delay_ + d);
    clean_buffer_trigger_.setPeriod(interruption_delay_ + buffer_horizon_);
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
  clean_buffer_trigger_.setPeriod(buffer_horizon_, false);
}

template <typename T>
void BufferedFunction<T>::cleanBuffer(const ros::TimerEvent& event)
{
  if (functions_.empty())
  {
    clean_buffer_trigger_.stop();
    return;
  }
  ros::Duration d(event.current_expected - start_timestamp_);
  ROS_WARN_STREAM("cleaning buffer after " << d.toSec() << " [s] after start.");
  functions_.erase(functions_.begin());
  interruption_functions_.erase(interruption_functions_.begin());
  ROS_WARN_STREAM("functions size: " << functions_.size()
                                     << ", interruption functions size: "
                                     << interruption_functions_.size());
  if (!functions_.empty())
  {
    Function<T>* function = functions_.front();
    d = ros::Duration(function->getDf()) - d + buffer_horizon_;
    clean_buffer_trigger_.setPeriod(d);
    ROS_WARN_STREAM("resetting clean buffer trigger to "
                    << d.toSec() << " [s] after start.");
    return;
  }
  start_timestamp_ = ros::Time::now();
  last_update_timestamp_ = start_timestamp_;
}
}
}

#endif // _UTILITIES_BUFFERED_FUNCTION_H_
