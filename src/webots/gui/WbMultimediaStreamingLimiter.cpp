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

#include "WbMultimediaStreamingLimiter.hpp"

#include "WbLog.hpp"

#include <QtCore/QObject>
#include <QtCore/QString>

WbMultimediaStreamingLimiter::WbMultimediaStreamingLimiter(const QSize &resolution, int updateTimeStep) :
  mResolution(resolution),
  mResolutionFactor(1),
  mResolutionChanged(false),
  mUpdateTimeStep(updateTimeStep),
  mLevel(-1),
  mIncreasingSteps(1),
  mIsStopped(false){};

void WbMultimediaStreamingLimiter::resetResolution(const QSize &newSize) {
  mResolution = newSize;
  for (int i = mResolutionFactor; i > 1; i--)
    mResolution /= 2;
}

QSize WbMultimediaStreamingLimiter::fullResolution() const {
  QSize resolutionSize(mResolution);
  for (int i = mResolutionFactor; i > 1; i--)
    resolutionSize *= 2;
  return resolutionSize;
}

void WbMultimediaStreamingLimiter::recomputeStreamingLimits(int skippedImages) {
  mIsStopped = false;
  mResolutionChanged = false;

  if (skippedImages > 0)
    mIncreasingSteps++;
  else if (mIncreasingSteps > 0)
    mIncreasingSteps--;

  if (mIncreasingSteps == 0) {
    if (mLevel >= 0) {
      mLevel--;
      if (mLevel < 6 && mLevel % 2 == 0 && mResolutionFactor > 1) {
        mResolution *= 2;
        mResolutionFactor--;
        mResolutionChanged = true;
      } else if (mUpdateTimeStep > 50)
        mUpdateTimeStep -= 50;
    }
  } else {
    if (skippedImages > 3)
      mIsStopped = true;
    while (skippedImages > 0) {
      if (mResolutionFactor < 3 && mLevel % 2 == 0) {
        mLevel++;
        mResolution /= 2;
        mResolutionFactor++;
        mResolutionChanged = true;
      } else if (mUpdateTimeStep < 500) {
        mLevel++;
        mUpdateTimeStep += 50;
      } else
        break;
      skippedImages -= 3;
    }
  }

  // Debug streaming limiter
  // WbLog::info(QString("Streaming status: resolution: %1x%2, rate %3")
  //              .arg(mResolution.width())
  //              .arg(mResolution.height())
  //              .arg(mUpdateTimeStep));
}
