#ifndef _ALLIANCE_LAYER_H_
#define _ALLIANCE_LAYER_H_

#include <string>

namespace alliance
{
class  Layer
{
public:
  Layer();
  Layer(const Layer& layer);
  virtual ~Layer();
  virtual void initialize(const std::string& name);
  virtual void process() = 0;
  std::string getName() const;
  bool operator==(const Layer& layer) const;
  bool operator!=(const Layer& layer) const;

private:
  std::string name_;
};
}

#endif // _ALLIANCE_LAYER_H_
