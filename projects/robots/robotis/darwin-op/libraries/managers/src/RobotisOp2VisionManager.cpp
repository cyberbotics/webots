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

#include "RobotisOp2VisionManager.hpp"

using namespace Robot;
using namespace managers;
using namespace std;

RobotisOp2VisionManager::RobotisOp2VisionManager(int width, int height, int hue, int hueTolerance, int minSaturation,
                                                 int minValue, int minPercent, int maxPercent) {
  mFinder = new ColorFinder(hue, hueTolerance, minSaturation, minValue, minPercent, maxPercent);
  mBuffer = new FrameBuffer(width, height);
}

RobotisOp2VisionManager::~RobotisOp2VisionManager() {
  delete mFinder;
  delete mBuffer;
}

bool RobotisOp2VisionManager::getBallCenter(double &x, double &y, const unsigned char *image) {
  Point2D pos;

  // Put the image in mBuffer
  mBuffer->m_BGRAFrame->m_ImageData = const_cast<unsigned char *>(image);
  // Convert the image from BGRA format to HSV format
  ImgProcess::BGRAtoHSV(mBuffer);
  // Extract position of the ball from HSV verson of the image
  pos = mFinder->GetPosition(mBuffer->m_HSVFrame);

  if (pos.X == -1 && pos.Y == -1) {
    x = 0.0;
    y = 0.0;
    return false;
  } else {
    x = pos.X;
    y = pos.Y;
    return true;
  }
}

bool RobotisOp2VisionManager::isDetected(int x, int y) {
  if (x > mFinder->m_result->m_Width || y > mFinder->m_result->m_Height)
    return false;

  int i = y * mFinder->m_result->m_Width + x;

  if (mFinder->m_result->m_ImageData[i] == 1)
    return true;
  else
    return false;
}
