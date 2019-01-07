/*
 * Description:  Abstraction of a motor device
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Device.hpp"

namespace webotsQtUtils {
  class Motor : public Device {
  public:
    explicit Motor(WbDeviceTag tag);
    virtual ~Motor() {}

    void enable(bool enable);
    bool isEnabled() const;
    double minPosition() const;
    double maxPosition() const;
    double position() const;
    double targetPosition() const;
    void setPosition(double position);
  };
}  // namespace webotsQtUtils

#endif
