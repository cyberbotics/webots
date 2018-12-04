/*
 * Description:  Abstraction of a LED
 */

#ifndef LED_HPP
#define LED_HPP

#include "Device.hpp"

class Led : public Device {
public:
  Led(WbDeviceTag tag, int index) : Device(tag, index), mState(false) {}
  virtual ~Led() {}

  int state() const { return mState; }
  void setState(int state) { mState = state; }

private:
  int mState;
};

#endif
