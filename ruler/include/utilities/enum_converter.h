/**
 *  This header file defines the EnumConverter abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_ENUM_CONVERTER_H_
#define _UTILITIES_ENUM_CONVERTER_H_

#include <string>
#include <vector>

namespace utilities
{

template <typename E> class EnumConverter
{
public:
  virtual ~EnumConverter();
  E getEnumerated() const;
  virtual E getEnumerated(int code) const = 0;
  virtual E getEnumerated(std::string name) const = 0;
  int getCode() const;
  virtual int getCode(std::string name) const = 0;
  virtual int getCode(E enumerated) const = 0;
  std::string str() const;
  virtual std::string str(E enumerated) const = 0;
  const char* c_str() const;
  virtual const char* c_str(E enumerated) const = 0;
  void operator=(int code);
  void operator=(std::string nome);
  void operator=(E enumerated);

protected:
  EnumConverter(E enumerated);

private:
  E enumerated_;
};

/**
 * The implementation is used inside the header file because of
 * the use of templates. This is the most recomended usage.
 * Benefits: General usage.
 */

/**
 *
 */
template <typename E>
EnumConverter<E>::EnumConverter(E enumerated)
    : enumerated_(enumerated)
{
}

/**
 *
 */
template <typename E> EnumConverter<E>::~EnumConverter() {}

/**
 *
 */
template <typename E> E EnumConverter<E>::getEnumerated() const
{
  return enumerated_;
}

/**
 *
 */
template <typename E> int EnumConverter<E>::getCode() const
{
  return getId(enumerated_);
}

/**
 *
 */
template <typename E> std::string EnumConverter<E>::str() const
{
  return str(enumerated_);
}

/**
 *
 */
template <typename E> const char* EnumConverter<E>::c_str() const
{
  return str().c_str();
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(int id)
{
  enumerated_ = getEnumerated(id);
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(std::string nome)
{
  enumerated_ = getEnumerated(nome);
}

/**
 *
 */
template <typename E> void EnumConverter<E>::operator=(E enumerated)
{
  enumerated_ = enumerated;
}
}

#endif // _UTILITIES_ENUM_CONVERTER_H_
