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

#include <webots/Camera.hpp>
#include <webots/Robot.hpp>

#include <LinuxDARwIn.h>

#include "Camera.h"

#include <iostream>

using namespace std;

unsigned char * ::webots::Camera::mImage = NULL;
const int ::webots::Camera::mResolution[NBRESOLUTION][2] = {{320, 240}, {640, 360}, {640, 400},
                                                            {640, 480}, {768, 480}, {800, 600}};

::webots::Camera::Camera(const string &name) : Device(name) {
  mIsActive = false;
}

::webots::Camera::~Camera() {
  disable();
}

void ::webots::Camera::enable(int samplingPeriod) {
  disable();
  ::Robot::LinuxCamera::GetInstance()->Initialize(0);
  ::Robot::LinuxCamera::GetInstance()->SetCameraSettings(::Robot::CameraSettings());
  mImage = static_cast<unsigned char *>(malloc(4 * getWidth() * getHeight()));

  int error = 0;

  // create and start the thread
  if ((error = pthread_create(&this->mCameraThread, NULL, this->CameraTimerProc, this)) != 0) {
    cerr << "Camera thread error = " << error << endl;
    exit(-1);
  }

  mIsActive = true;
}

void ::webots::Camera::disable() {
  if (mIsActive) {
    // End the thread
    if (pthread_cancel(this->mCameraThread) != 0)
      exit(-1);
    mIsActive = false;
  }
  if (mImage) {
    free(mImage);
    mImage = NULL;
  }
}

const unsigned char * ::webots::Camera::getImage() const {
  return mImage;
}

void * ::webots::Camera::CameraTimerProc(void *param) {
  while (1) {
    ::Robot::LinuxCamera::GetInstance()->CaptureFrameWb();
    memcpy(mImage, ::Robot::LinuxCamera::GetInstance()->fbuffer->m_BGRAFrame->m_ImageData,
           ::Robot::LinuxCamera::GetInstance()->fbuffer->m_BGRAFrame->m_ImageSize);
  }
  return NULL;
}

int ::webots::Camera::getWidth() const {
  return ::Robot::Camera::WIDTH;
}

int ::webots::Camera::getHeight() const {
  return ::Robot::Camera::HEIGHT;
}

double ::webots::Camera::getFov() const {
  return 1.0123;
}

double ::webots::Camera::getNear() const {
  return 0.0;
}

unsigned char ::webots::Camera::imageGetRed(const unsigned char *image, int width, int x, int y) {
  return image[3 * (y * width + x)];
}

unsigned char ::webots::Camera::imageGetGreen(const unsigned char *image, int width, int x, int y) {
  return image[3 * (y * width + x) + 1];
}

unsigned char ::webots::Camera::imageGetBlue(const unsigned char *image, int width, int x, int y) {
  return image[3 * (y * width + x) + 2];
}

unsigned char ::webots::Camera::imageGetGray(const unsigned char *image, int width, int x, int y) {
  return image[3 * (y * width + x)] / 3 + image[3 * (y * width + x) + 1] / 3 + image[3 * (y * width + x) + 2] / 3;
}

bool ::webots::Camera::checkResolution(int width, int height) {
  for (int i = 0; i < NBRESOLUTION; i++) {
    if ((mResolution[i][0] == width) && (mResolution[i][1] == height))
      return true;
  }
  return false;
}

int ::webots::Camera::getSamplingPeriod() const {
  if (Robot::getInstance()->getBasicTimeStep() < 30)
    return 30;
  else
    return Robot::getInstance()->getBasicTimeStep();
}

int ::webots::Camera::getType() const {
  return WB_CAMERA_COLOR;
}
