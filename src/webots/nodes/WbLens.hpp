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

#ifndef WB_LENS_HPP
#define WB_LENS_HPP

#include "WbBaseNode.hpp"
#include "WbSFVector2.hpp"

class WbLens : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbLens(WbTokenizer *tokenizer = NULL);
  WbLens(const WbLens &other);
  explicit WbLens(const WbNode &other);
  virtual ~WbLens();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LENS; }
  void preFinalize() override;
  void postFinalize() override;

  // getters
  const WbVector2 &center() const { return mCenter->value(); }
  const WbVector2 &radialCoefficients() const { return mRadialCoefficients->value(); }
  const WbVector2 &tangentialCoefficients() const { return mTangentialCoefficients->value(); }

signals:
  void centerChanged();
  void radialCoefficientsChanged();
  void tangentialCoefficientsChanged();

private:
  WbLens &operator=(const WbLens &);  // non copyable
  WbNode *clone() const override { return new WbLens(*this); }

  void init();

  WbSFVector2 *mCenter;
  WbSFVector2 *mRadialCoefficients;
  WbSFVector2 *mTangentialCoefficients;

private slots:
  void updateCenter();
  void updateRadialCoefficients();
  void updateTangentialCoefficients();
};

#endif
