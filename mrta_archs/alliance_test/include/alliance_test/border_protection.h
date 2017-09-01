#ifndef _ALLIANCE_TEST_BORDER_PROTECTION_H_
#define _ALLIANCE_TEST_BORDER_PROTECTION_H_

#include "alliance_test/layer.h"
#include <sensor_msgs/PointCloud.h>

namespace alliance_test
{
class BorderProtection : public Layer
{
public:
  BorderProtection();
  virtual ~BorderProtection();
  virtual void initialize(const std::string& name);
  virtual void process();

private:
  nodes::ROSSensorMessage<sensor_msgs::PointCloud>* sonars_;
};
}

#endif // _ALLIANCE_TEST_BORDER_PROTECTION_H_
