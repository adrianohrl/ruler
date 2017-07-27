/**
 * This header file defines the HasId class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_HAS_ID_H_
#define _UTILITIES_HAS_ID_H_

#include <sstream>

namespace utilities
{
template <typename K, typename T> class HasId
{
public:
  virtual ~HasId();
  K getId() const;
  std::string str() const;
  const char* c_str() const;
  bool operator==(const HasId<K, T>& has_id);
  bool operator!=(const HasId<K, T>& has_id);
  template <typename U, typename V>
  friend std::ostream& operator<<(std::ostream& out,
                                  const HasId<U, V>& signal_type);

protected:
  HasId(const K& id);
  HasId(const HasId<K, T>& has_id);

private:
  const K id_;
};

template <typename K, typename T> HasId<K, T>::HasId(const K& id) : id_(id) {}

template <typename K, typename T>
HasId<K, T>::HasId(const HasId<K, T>& has_id)
    : id_(has_id.id_)
{
}

template <typename K, typename T> HasId<K, T>::~HasId() {}

template <typename K, typename T> K HasId<K, T>::getId() const { return id_; }

template <typename K, typename T> std::string HasId<K, T>::str() const
{
  std::stringstream ss;
  ss << id_;
  return ss.str();
}

template <typename K, typename T>
const char* HasId<K, T>::c_str() const { return str().c_str(); }

template <typename K, typename T>
bool HasId<K, T>::operator==(const HasId<K, T>& has_id)
{
  return id_ == has_id.id_;
}

template <typename K, typename T>
bool HasId<K, T>::operator!=(const HasId<K, T>& has_id)
{
  return id_ != has_id.id_;
}

template <typename K, typename T>
std::ostream& operator<<(std::ostream& os, const HasId<K, T>& has_id)
{
  os << has_id.id_;
  return os;
}
}

#endif // _UTILITIES_HAS_ID_H_
