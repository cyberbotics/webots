// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WB_PBR_APPEARANCE_HPP
#define WB_PBR_APPEARANCE_HPP

#include "WbAbstractAppearance.hpp"
#include "WbRgb.hpp"

class WbCubemap;
class WbImageTexture;

struct WrMaterial;

class WbPbrAppearance : public WbAbstractAppearance {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbPbrAppearance(WbTokenizer *tokenizer = NULL);
  WbPbrAppearance(const WbPbrAppearance &other);
  explicit WbPbrAppearance(const WbNode &other);
  virtual ~WbPbrAppearance();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PBR_APPEARANCE; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset() override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override { return true; }

  void setEmissiveColor(const WbRgb &color);

  const WbRgb &initialEmissiveColor() const { return mInitialEmissiveColor; }

  // field accessors
  WbImageTexture *baseColorMap() const;
  WbImageTexture *roughnessMap() const;
  WbImageTexture *metalnessMap() const;
  WbCubemap *environmentMap() const;
  WbImageTexture *normalMap() const;
  WbImageTexture *occlusionMap() const;
  WbImageTexture *emissiveColorMap() const;

  // Inherited from WbAbstractAppearance
  WrMaterial *modifyWrenMaterial(WrMaterial *wrenMaterial) override;

  // specific functions
  bool isBaseColorTextureLoaded() const;
  void pickColorInBaseColorTexture(WbRgb &pickedColor, const WbVector2 &uv) const;
  WbRgb baseColor() const;
  double transparency() const;

protected:
  void exportNodeSubNodes(WbVrmlWriter &writer) const override;

private:
  WbPbrAppearance &operator=(const WbPbrAppearance &);  // non copyable
  WbNode *clone() const override { return new WbPbrAppearance(*this); }
  void clearCubemap(WrMaterial *wrenMaterial);

  bool isTransparent();
  void init();

  WbSFColor *mBaseColor;
  WbSFNode *mBaseColorMap;
  WbSFDouble *mTransparency;
  WbSFDouble *mRoughness;
  WbSFNode *mRoughnessMap;
  WbSFDouble *mMetalness;
  WbSFNode *mMetalnessMap;
  WbSFNode *mEnvironmentMap;
  WbSFDouble *mIblStrength;
  WbSFNode *mNormalMap;
  WbSFDouble *mNormalMapFactor;
  WbSFNode *mOcclusionMap;
  WbSFDouble *mOcclusionMapStrength;
  WbSFColor *mEmissiveColor;
  WbSFNode *mEmissiveColorMap;
  WbSFDouble *mEmissiveIntensity;

  WbRgb mInitialEmissiveColor;

private slots:
  void updateCubeMap();
  void updateBackgroundColor();
  void updateBaseColor();
  void updateBaseColorMap();
  void updateTransparency();
  void updateRoughness();
  void updateRoughnessMap();
  void updateMetalness();
  void updateMetalnessMap();
  void updateEnvironmentMap();
  void updateIblStrength();
  void updateNormalMap();
  void updateNormalMapFactor();
  void updateOcclusionMap();
  void updateOcclusionMapStrength();
  void updateEmissiveColor();
  void updateEmissiveColorMap();
  void updateEmissiveIntensity();
};

#endif
