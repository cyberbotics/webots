/*
 * Description:  Abstraction of a camera
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "Sensor.hpp"

#include <string>

class Camera : public Sensor {
public:
  // Device Manager is responsible to create/destroy devices
  explicit Camera(WbDeviceTag tag);
  virtual ~Camera() {}

  int width() const { return mWidth; }
  int height() const { return mHeight; }
  bool rawToBgraImage(unsigned char *bgraImage, const unsigned char *rawImage) const;

private:
  int mWidth;
  int mHeight;
};

#endif
