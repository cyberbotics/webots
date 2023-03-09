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

#include <QtCore/QSize>

class WbMultimediaStreamingLimiter {
public:
  WbMultimediaStreamingLimiter(const QSize &resolution, int updateTimeStep);

  void recomputeStreamingLimits(int skippedImages);

  void resetResolution(const QSize &newSize);
  void resetStop() { mIsStopped = false; }

  const QSize &resolution() const { return mResolution; }
  int resolutionFactor() const { return mResolutionFactor; }
  bool resolutionChanged() const { return mResolutionChanged; }
  QSize fullResolution() const;
  int updateTimeStep() const { return mUpdateTimeStep; }
  bool isStopped() const { return mIsStopped; }

private:
  QSize mResolution;
  int mResolutionFactor;
  bool mResolutionChanged;
  int mUpdateTimeStep;
  int mLevel;
  int mIncreasingSteps;
  bool mIsStopped;
};
