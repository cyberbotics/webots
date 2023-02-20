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

#ifndef WB_FOCUS_HPP
#define WB_FOCUS_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"

class WbFocus : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbFocus(WbTokenizer *tokenizer = NULL);
  WbFocus(const WbFocus &other);
  explicit WbFocus(const WbNode &other);
  virtual ~WbFocus();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_FOCUS; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  double focalDistance() const { return mFocalDistance->value(); }
  double focalLength() const { return mFocalLength->value(); }
  double minFocalDistance() const { return mMinFocalDistance->value(); }
  double maxFocalDistance() const { return mMaxFocalDistance->value(); }

  // setters
  void setFocalDistance(double focalDistance) { mFocalDistance->setValue(focalDistance); }

signals:
  void focusSettingsChanged();

private:
  WbFocus &operator=(const WbFocus &);  // non copyable
  WbNode *clone() const override { return new WbFocus(*this); }

  void init();

  WbSFDouble *mFocalDistance;
  WbSFDouble *mFocalLength;
  WbSFDouble *mMinFocalDistance;
  WbSFDouble *mMaxFocalDistance;

private slots:
  void updateFocalDistance();
  void updateFocalLength();
  void updateMinFocalDistance();
  void updateMaxFocalDistance();
};

#endif
