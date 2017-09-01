#include "alliance/layer.h"
#include <utilities/exception.h>

namespace alliance
{
Layer::Layer() {}

Layer::Layer(const Layer& layer) : name_(layer.name_) {}

Layer::~Layer() {}

void Layer::initialize(const std::string& name)
{
  if (name.empty())
  {
    throw utilities::Exception("Layer name must not be empty.");
  }
  name_ = name;
}

std::string Layer::getName() const
{
  return name_;
}

bool Layer::operator==(const Layer& layer) const
{
  return name_ == layer.name_;
}

bool Layer::operator!=(const Layer& layer) const
{
  return name_ != layer.name_;
}
}
