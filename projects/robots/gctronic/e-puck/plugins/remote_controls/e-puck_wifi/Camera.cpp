#include "Camera.hpp"

#include <webots/camera.h>

#include <iostream>
#include <sstream>

using namespace std;

Camera::Camera(WbDeviceTag tag) : Sensor(tag, 0) {
  mWidth = wb_camera_get_width(tag);
  mHeight = wb_camera_get_height(tag);
}

bool Camera::rawToBgraImage(unsigned char *bgraImage, const unsigned char *rawImage) const {
  int index = 0;
  int counter = 0;
  unsigned char red, green, blue;

  for (int j = 0; j < mHeight; j++) {
    for (int i = 0; i < mWidth; i++) {
      index = 4 * (i + j * mWidth);

      red = rawImage[counter] & 0xf8;
      green = (rawImage[counter++] << 5);
      green += (rawImage[counter] & 0xf8) >> 3;
      blue = rawImage[counter++] << 3;

      bgraImage[index] = blue;
      bgraImage[index + 1] = green;
      bgraImage[index + 2] = red;
      bgraImage[index + 3] = 0xff;
    }
  }
  return true;
}
