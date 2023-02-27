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

#ifndef WB_RANGE_FINDER_HPP
#define WB_RANGE_FINDER_HPP

#include "WbAbstractCamera.hpp"

class WbRangeFinder : public WbAbstractCamera {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbRangeFinder(WbTokenizer *tokenizer = NULL);
  WbRangeFinder(const WbRangeFinder &other);
  explicit WbRangeFinder(const WbNode &other);
  virtual ~WbRangeFinder();

  // reimplemented public functions
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  int nodeType() const override { return WB_NODE_RANGE_FINDER; }
  QString pixelInfo(int x, int y) const override;
  WbRgb enabledCameraFrustrumColor() const override { return WbRgb(1.0f, 1.0f, 0.0f); }

  bool isRangeFinder() override { return true; }
  double maxRange() const override { return mMaxRange->value(); }

  int textureGLId() const override;

private:
  // user accessible fields
  WbSFDouble *mMinRange;
  WbSFDouble *mMaxRange;
  WbSFDouble *mResolution;

  // private functions
  void addConfigureToStream(WbDataStream &stream, bool reconfigure = false) override;

  float *rangeFinderImage() const;

  WbRangeFinder &operator=(const WbRangeFinder &);  // non copyable
  WbNode *clone() const override { return new WbRangeFinder(*this); }
  void init();
  void initializeImageMemoryMappedFile() override;

  int size() const override { return sizeof(float) * width() * height(); }
  double minRange() const override { return mMinRange->value(); }
  bool isFrustumEnabled() const override;

  // WREN
  void createWrenCamera() override;
  void applyMinRangeToWren();
  void applyMaxRangeToWren();
  void applyResolutionToWren();

private slots:
  void updateNear();
  void updateMinRange();
  void updateMaxRange();
  void updateResolution();
  void updateOrientation();
  void applyCameraSettingsToWren() override;
  void updateFrustumDisplayIfNeeded(int optionalRendering) override;
};

#endif  // WB_RANGE_FINDER_HPP
