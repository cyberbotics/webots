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

#ifndef WB_MATERIAL_HPP
#define WB_MATERIAL_HPP

#include "WbBaseNode.hpp"
#include "WbRgb.hpp"

struct WrMaterial;

class WbMaterial : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbMaterial(WbTokenizer *tokenizer = NULL);
  WbMaterial(const WbMaterial &other);
  explicit WbMaterial(const WbNode &other);
  virtual ~WbMaterial();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MATERIAL; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  float transparency() const;
  const WbRgb &emissiveColor() const;
  void setEmissiveColor(const WbRgb &color);
  const WbRgb &diffuseColor() const;

  const WbRgb &initialEmissiveColor() const { return mInitialEmissiveColor; }

  void modifyWrenMaterial(WrMaterial *wrenMaterial, bool textured);

  // export
  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void changed();

private:
  // user accessible fields
  WbSFDouble *mAmbientIntensity;
  WbSFColor *mDiffuseColor;
  WbSFColor *mEmissiveColor;
  WbSFDouble *mShininess;
  WbSFColor *mSpecularColor;
  WbSFDouble *mTransparency;

  WbRgb mInitialEmissiveColor;

  WbMaterial &operator=(const WbMaterial &);  // non copyable
  WbNode *clone() const override { return new WbMaterial(*this); }

  void init();

private slots:
  void updateAmbientIntensity();
  void updateDiffuseColor();
  void updateEmissiveColor();
  void updateShininess();
  void updateSpecularColor();
  void updateTransparency();
};

#endif
