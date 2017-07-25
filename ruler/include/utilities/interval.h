/**
 * This header file defines the Interval class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_INTERVAL_H_
#define _UTILITIES_INTERVAL_H_

namespace utilities
{
template <typename T> class Interval
{
public:
  Interval(const T& min, const T& max, bool including_min = true,
           bool including_max = true);
  virtual ~Interval();
  bool belongs(const T& value) const;
  T getValid(const T& value) const;
  T getMin() const;
  T getMax() const;

private:
  const T min_;
  const T max_;
  bool including_min_;
  bool inculding_max_;
};

template <typename T>
Interval<T>::Interval(const T& min, const T& max, bool including_min,
                      bool including_max)
    : min_(min), max_(max), including_min_(including_min),
      inculding_max_(including_max)
{
}

template <typename T> Interval<T>::~Interval() {}

template <typename T> bool Interval<T>::belongs(const T& value) const
{
  return including_min_ ? value >= min_ : value > min_ && inculding_max_
                                              ? value <= max_
                                              : value < max_;
}

template <typename T> T Interval<T>::getValid(const T& value) const
{
  return value < min_ ? min_ : value > max_ ? max_ : value;
}

template <typename T> T Interval<T>::getMin() const { return min_; }

template <typename T> T Interval<T>::getMax() const { return max_; }
}

#endif // _UTILITIES_INTERVAL_H_
