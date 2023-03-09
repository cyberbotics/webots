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

#ifndef WB_PEN_HPP
#define WB_PEN_HPP

#include "WbSolidDevice.hpp"

class WbPaintTexture;

class WbPen : public WbSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbPen(WbTokenizer *tokenizer = NULL);
  WbPen(const WbPen &other);
  explicit WbPen(const WbNode &other);
  virtual ~WbPen();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PEN; }
  void createWrenObjects() override;
  void preFinalize() override;
  void handleMessage(QDataStream &) override;
  void prePhysicsStep(double ms) override;
  void reset(const QString &id) override;

private:
  WbSFBool *mWrite;
  WbSFColor *mInkColor;
  WbSFDouble *mInkDensity;
  WbSFDouble *mLeadSize;
  WbSFDouble *mMaxDistance;

  // Optional rendering
  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  WbPen &operator=(const WbPen &);  // non copyable
  WbNode *clone() const override { return new WbPen(*this); }
  void init();

  WbPaintTexture *mLastPaintTexture;

private slots:
  void applyOptionalRenderingToWren();
  void updateOptionalRendering(int option);
};

#endif  // WB_PEN_HPP
