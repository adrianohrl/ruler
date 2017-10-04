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
};
}

#endif // _ALLIANCE_TEST_BORDER_PROTECTION_H_
