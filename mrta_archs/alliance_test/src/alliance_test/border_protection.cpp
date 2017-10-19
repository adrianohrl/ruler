#include "alliance_test/border_protection.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(alliance_test::BorderProtection, alliance::Layer)

namespace alliance_test
{
BorderProtection::BorderProtection() : align_left_(false), align_right_(false)
{
}

BorderProtection::~BorderProtection() {}

void BorderProtection::process()
{
  Layer::process();
  double vx(0.0), wz(0.0);
  if (odometry_->getX() < 9.0 && odometry_->getX() > -7.0 &&
      odometry_->getY() < 6.0 &&
      odometry_->getY() > -6.0) // ler tamanho do map por parametros
  {
    vx =
        GAIN * (std::max(sonars_->getDistance(2), sonars_->getDistance(3)) -
                1 / std::max(sonars_->getDistance(2), sonars_->getDistance(3)));
    wz = 0.5 * GAIN /
         std::min(sonars_->getDistance(2), sonars_->getDistance(3)) *
         (std::min(sonars_->getDistance(0),
                   std::min(sonars_->getDistance(1), sonars_->getDistance(2))) -
          std::min(sonars_->getDistance(3),
                   std::min(sonars_->getDistance(4), sonars_->getDistance(5))));
  }
  else if (sonars_->getDistance(0) < sonars_->getDistance(5) && align_left_)
  {
    align_right_ = false;
    vx = 0.5;
    wz = 0.4 * (sonars_->getDistance(0) - DANGEROUS_DISTANCE) +
         1.2 * (sonars_->getDistance(1) - 0.75 * sonars_->getDistance(7)) +
         0.3 * (sonars_->getDistance(3) -
                std::max(sonars_->getDistance(4),
                         std::max(sonars_->getDistance(6),
                                  sonars_->getDistance(5))));
  }
  else if (sonars_->getDistance(5) < sonars_->getDistance(0) && align_right_)
  {
    align_left_ = false;
    vx = 0.5;
    wz = -0.4 * (sonars_->getDistance(5) - DANGEROUS_DISTANCE) -
         1.2 * (sonars_->getDistance(4) - 0.75 * sonars_->getDistance(6)) -
         0.3 * (sonars_->getDistance(2) -
                std::max(sonars_->getDistance(1),
                         std::max(sonars_->getDistance(7),
                                  sonars_->getDistance(0))));
  }
  Layer::setVelocity(vx, wz);
}
}
