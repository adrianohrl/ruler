#ifndef _ALLIANCE_TEST_WANDER_H_
#define _ALLIANCE_TEST_WANDER_H_

#include "alliance_test/layer.h"

namespace alliance_test
{
class Wander : public Layer
{
public:
  Wander();
  virtual ~Wander();
  virtual void process();
};
}

#endif // _ALLIANCE_TEST_WANDER_H_
