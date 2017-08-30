#ifndef _UTILITIES_SAMPLE_HOLDER_H_
#define _UTILITIES_SAMPLE_HOLDER_H_

#include "utilities/functions/buffered_function.h"
#include "utilities/functions/step_function.h"
#include "utilities/functions/value_change_event.h"

namespace utilities
{
namespace functions
{
template <typename T> class SampleHolder : public BufferedFunction<T>
{
public:
  SampleHolder(const std::string& id, StepFunction<T>* model,
               const ros::Duration& timeout_duration,
               const ros::Duration& buffer_horizon,
               const ros::Time& start_timestamp = ros::Time::now());
  SampleHolder(const SampleHolder<T>& sample_holder);
  virtual ~SampleHolder();
  void update(ValueChangeEvent<T>* event);
};

template <typename T>
SampleHolder<T>::SampleHolder(const std::string& id, StepFunction<T>* model,
                              const ros::Duration& timeout_duration,
                              const ros::Duration& buffer_horizon,
                              const ros::Time& start_timestamp)
    : BufferedFunction<T>::BufferedFunction(id, model, timeout_duration,
                                            buffer_horizon, start_timestamp)
{
}

template <typename T>
SampleHolder<T>::SampleHolder(const SampleHolder<T>& sample_holder)
    : BufferedFunction<T>::BufferedFunction(sample_holder)
{
}

template <typename T> SampleHolder<T>::~SampleHolder() {}

template <typename T> void SampleHolder<T>::update(ValueChangeEvent<T> *event)
{
  BufferedFunction<T>::update(event,
                              new StepFunction<T>(event->getValue(), true));
}
}
}

#endif // _UTILITIES_SAMPLE_HOLDER_H_
