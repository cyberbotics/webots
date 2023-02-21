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
  mLinearCamera = mHeight == 1;

  computeZoom();
}

void Camera::checkResolution() const {
  if (mWidth > 127)
    cerr << "Camera resolution problem: the width must be smaller than 128" << endl;
  if (mHeight > 127)
    cerr << "Camera resolution problem: the height must be smaller than 128" << endl;
  if (mWidth * mHeight > 52 * 39)
    cerr << "Camera resolution problem: the resolution " << mWidth << "x" << mHeight << " is bigger than the buffer size"
         << endl;
}

std::string Camera::generateInitialisationCommand() const {
  stringstream s;
  int x = -1;
  int y = -1;
  if (mLinearCamera) {
    x = (480 - width() * mZoom) / 2;
    y = (480 - height() * mZoom) / 2;
  }
  s << "J,1," << width() << "," << height() << "," << mZoom << "," << x << "," << y << "\n";
  return s.str();
}

void Camera::computeZoom() {
  if (mLinearCamera)
    mZoom = 480 / mWidth;
  else {
    if ((double)mWidth / 480 > (double)mHeight / 640) {
      // dynamic zoom
      mZoom = 480 / mWidth;
      while (mZoom * mWidth > 480)
        mZoom--;

      // search the biggest power of 2 below the value
      int iter = 0;
      while (mZoom > 1) {
        mZoom /= 2;
        iter++;
      }
      mZoom = 1 << iter;
      if (mZoom < 1)
        mZoom = 1;
    } else {
      // dynamic zoom
      mZoom = 640 / mHeight;
      while (mZoom * mHeight > 640)
        mZoom--;

      // search the biggest power of 2 below the value
      int iter = 0;
      while (mZoom > 1) {
        mZoom /= 2;
        iter++;
      }
      mZoom = 1 << iter;
      if (mZoom < 1)
        mZoom = 1;
    }
  }
}

bool Camera::rawToBgraImage(unsigned char *bgraImage, const unsigned char *rawImage, int mode, int wh1, int wh2) const {
  int w = wh1;
  int h = wh2;

  if (mode != 1 || this->width() != w || this->height() != h) {
    cerr << "The received image doesn't match with the initialization sent" << endl;
    return false;
  }

  int index;
  int counter = 0;
  unsigned char red, green, blue;

  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      index = 4 * (i + j * w);

      red = rawImage[counter] & 0xf8;
      green = (rawImage[counter++] << 5);
      green += (rawImage[counter] & 0xf8) >> 3;
      blue = rawImage[counter++] << 3;

      bgraImage[index] = blue;
      bgraImage[index + 1] = green;
      bgraImage[index + 2] = red;
      bgraImage[index + 3] = 0xFF;
    }
  }

  return true;
}
