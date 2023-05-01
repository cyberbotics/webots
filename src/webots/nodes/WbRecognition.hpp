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

#ifndef WB_RECOGNITION_HPP
#define WB_RECOGNITION_HPP

#include "WbBaseNode.hpp"
#include "WbSFBool.hpp"
#include "WbSFColor.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"

class WbRecognition : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbRecognition(WbTokenizer *tokenizer = NULL);
  WbRecognition(const WbRecognition &other);
  explicit WbRecognition(const WbNode &other);
  virtual ~WbRecognition();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RECOGNITION; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double maxRange() const { return mMaxRange->value(); }
  int maxObjects() const { return mMaxObjects->value(); }
  int occlusion() const { return mOcclusion->value(); }
  const WbRgb frameColor() const { return mFrameColor->value(); }
  int frameThickness() const { return mFrameThickness->value(); }
  bool segmentation() const { return mSegmentation->value(); }

  void setSegmentation(bool value) { mSegmentation->setValue(value); }

signals:
  void segmentationChanged();

private:
  WbRecognition &operator=(const WbRecognition &);  // non copyable
  WbNode *clone() const override { return new WbRecognition(*this); }

  void init();

  WbSFDouble *mMaxRange;
  WbSFInt *mMaxObjects;
  WbSFInt *mOcclusion;
  WbSFColor *mFrameColor;
  WbSFInt *mFrameThickness;
  WbSFBool *mSegmentation;

private slots:
  void updateMaxRange();
  void updateMaxObjects();
  void updateOcclusion();
  void updateFrameThickness();
};

#endif
