#ifndef _SENSORS_SONAR_H_
#define _SENSORS_SONAR_H_

#include <nodes/ros_sensor_message.h>
#include <sensor_msgs/PointCloud.h>

namespace sensors
{
class Sonar : public nodes::ROSSensorMessage<sensor_msgs::PointCloud>
{
public:
  typedef std::vector<geometry_msgs::Point32>::iterator iterator;
  typedef std::vector<geometry_msgs::Point32>::const_iterator const_iterator;
  Sonar(const std::string& id, const ros::NodeHandlePtr& nh,
        const std::string& ns, const std::string& topic_name = "sonar",
        const ros::Duration& timeout_duration = ros::Duration(1.0));
  virtual ~Sonar();
  double operator[](int index) const;
  double getDistance(int index) const;
  geometry_msgs::Point32 getPoint(int index) const;
  bool empty() const;
  std::size_t size() const;
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;
};

typedef boost::shared_ptr<Sonar> SonarPtr;
typedef boost::shared_ptr<Sonar const> SonarConstPtr;
}

#endif // _SENSORS_SONAR_H_
