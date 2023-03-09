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

#ifndef WB_APPEARANCE_HPP
#define WB_APPEARANCE_HPP

#include "WbAbstractAppearance.hpp"

class WbImageTexture;
class WbMaterial;
class WbRgb;
class WbVector2;

struct WrMaterial;

class WbAppearance : public WbAbstractAppearance {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbAppearance(WbTokenizer *tokenizer = NULL);
  WbAppearance(const WbAppearance &other);
  explicit WbAppearance(const WbNode &other);
  virtual ~WbAppearance();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_APPEARANCE; }
  void downloadAssets() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override { return true; }

  // field accessors
  WbMaterial *material() const;
  WbImageTexture *texture() const;

  // Inherited from WbAbstractAppearance
  WrMaterial *modifyWrenMaterial(WrMaterial *wrenMaterial) override;

  // specific functions
  bool isTextureLoaded() const;
  void pickColorInTexture(const WbVector2 &uv, WbRgb &pickedColor) const;
  WbRgb diffuseColor() const;

  static WrMaterial *fillWrenDefaultMaterial(WrMaterial *wrenMaterial);

private:
  // user accessible fields
  WbSFNode *mMaterial;
  WbSFNode *mTexture;

  WbAppearance &operator=(const WbAppearance &);  // non copyable
  WbNode *clone() const override { return new WbAppearance(*this); }

  void init();

private slots:
  void updateMaterial();
  void updateTexture();
};

#endif
