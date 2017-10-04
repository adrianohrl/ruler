#ifndef _ALLIANCE_LAYER_H_
#define _ALLIANCE_LAYER_H_

#include <boost/shared_ptr.hpp>
#include <string>

namespace alliance
{
class Layer
{
public:
  Layer();
  virtual ~Layer();
  virtual void initialize(const std::string& ns, const std::string& name);
  virtual void readParameters();
  virtual void process() = 0;
  std::string getName() const;
  bool operator==(const Layer& layer) const;
  bool operator!=(const Layer& layer) const;

private:
  std::string name_;
};

typedef boost::shared_ptr<Layer> LayerPtr;
typedef boost::shared_ptr<Layer const> LayerConstPtr;
}

#endif // _ALLIANCE_LAYER_H_
