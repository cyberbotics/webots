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
  void checkResolution() const;

  std::string generateInitialisationCommand() const;
  bool rawToBgraImage(unsigned char *bgraImage, const unsigned char *rawImage, int mode, int wh1, int wh2) const;

private:
  int mWidth;
  int mHeight;

  bool mLinearCamera;
  int mZoom;

  void computeZoom();
};

#endif
