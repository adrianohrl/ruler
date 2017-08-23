#ifndef _ALLIANCE_SENSOR_H_
#define _ALLIANCE_SENSOR_H_

namespace alliance
{
template <typename M>
class Sensor
{
public:
  Sensor(const Sensor& sensor);
  virtual ~Sensor();

private:
};
}

#endif // _ALLIANCE_SENSOR_H_
