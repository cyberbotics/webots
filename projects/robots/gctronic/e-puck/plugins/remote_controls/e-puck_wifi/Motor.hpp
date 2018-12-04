/*
 * Description:  Abstraction of a Motor
 */

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "Device.hpp"

class Motor : public Device {
public:
  // Device Manager is responsible to create/destroy devices
  Motor(WbDeviceTag tag, int index) : Device(tag, index), mVelocity(false) {}
  virtual ~Motor() {}

  double velocity() const { return mVelocity; }
  void setVelocity(double velocity) { mVelocity = velocity; }

private:
  double mVelocity;
};

#endif
