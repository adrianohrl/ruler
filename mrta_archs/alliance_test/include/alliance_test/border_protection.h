#ifndef _ALLIANCE_TEST_BORDER_PROTECTION_H_
#define _ALLIANCE_TEST_BORDER_PROTECTION_H_

#include "alliance_test/layer.h"

namespace alliance_test
{
class BorderProtection : public Layer
{
public:
  BorderProtection();
  virtual ~BorderProtection();
  virtual void process();

private:
  bool align_left_;
  bool align_right_;
  static const double DANGEROUS_DISTANCE = 0.5;
  static const double GAIN = 0.5;
};
}

#endif // _ALLIANCE_TEST_BORDER_PROTECTION_H_
