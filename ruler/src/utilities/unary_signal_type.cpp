/**
 *  This header file implements the UnarySignalType class, which is based on the
 *SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/unary_signal_type.h"

namespace utilities
{

UnarySignalType::UnarySignalType(bool value)
    : SignalType<bool>::SignalType(value)
{
}

UnarySignalType::UnarySignalType(const UnarySignalType& signal_type)
    : SignalType<bool>::SignalType(signal_type)
{
}

UnarySignalType::~UnarySignalType() {}

std::string UnarySignalType::str() const { return value_ ? "true" : "false"; }

bool UnarySignalType::operator!() const { return !value_; }
}
