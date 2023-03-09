// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  int index;
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
