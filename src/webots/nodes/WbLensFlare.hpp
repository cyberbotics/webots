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

#ifndef WB_LENS_FLARE_HPP
#define WB_LENS_FLARE_HPP

#include <QtCore/QMap>

#include "WbBaseNode.hpp"
#include "WbVector3.hpp"

class WbWrenLensFlare;
class WbSFDouble;

struct WrViewport;

class WbLensFlare : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbLensFlare(WbTokenizer *tokenizer = NULL);
  WbLensFlare(const WbLensFlare &other);
  explicit WbLensFlare(const WbNode &other);
  virtual ~WbLensFlare();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LENS; }
  void postFinalize() override;

  void setup(WrViewport *viewport);
  void detachFromViewport();

private:
  WbLensFlare &operator=(const WbLensFlare &);  // non copyable
  WbNode *clone() const override { return new WbLensFlare(*this); }

  void init();

  // user accessible fields
  WbSFDouble *mTransparency;
  WbSFDouble *mScale;
  WbSFDouble *mBias;
  WbSFDouble *mDispersal;
  WbSFDouble *mHaloWidth;
  WbSFDouble *mChromaDistortion;
  WbSFInt *mSamples;
  WbSFInt *mBlurIterations;

  QMap<WrViewport *, WbWrenLensFlare *> mWrenLensFlares;  // list of lens-flares with the associated viewport as key

private slots:
  void updateTransparency();
  void updateScale();
  void updateBias();
  void updateDispersal();
  void updateHaloWidth();
  void updateChromaDistortion();
  void updateSamples();
  void updateBlur();
};

#endif
