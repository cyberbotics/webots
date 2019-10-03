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

#ifndef WB_BACKGROUND_HPP
#define WB_BACKGROUND_HPP

#include "WbBaseNode.hpp"
#include "WbSFDouble.hpp"

class WbRgb;

struct WrTextureCubeMap;
struct WrShaderProgram;
struct WrRenderable;
struct WrMaterial;
struct WrStaticMesh;

class WbBackground : public WbBaseNode {
  Q_OBJECT

public:
  static WbBackground *firstInstance() { return cBackgroundList.isEmpty() ? NULL : cBackgroundList.first(); }
  static int numberOfBackgroundInstances() { return cBackgroundList.count(); }

  // constructors and destructor
  explicit WbBackground(WbTokenizer *tokenizer = NULL);
  WbBackground(const WbBackground &other);
  explicit WbBackground(const WbNode &other);
  virtual ~WbBackground();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_BACKGROUND; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;

  // accessor
  WbRgb skyColor() const;
  double luminosity() const { return mLuminosity->value(); }

  WrTextureCubeMap *diffuseIrradianceCubeTexture() { return mDiffuseIrradianceCubeTexture; };
  WrTextureCubeMap *specularIrradianceCubeTexture() { return mSpecularIrradianceCubeTexture; };

signals:
  void cubemapChanged();
  void luminosityChanged();

protected:
  void exportNodeFields(WbVrmlWriter &writer) const override;

private:
  static QList<WbBackground *> cBackgroundList;

  // reimplemented functions
  WbNode *clone() const override { return new WbBackground(*this); }

  // misc functions
  WbBackground &operator=(const WbBackground &);  // non copyable
  void init();
  void destroySkyBox();
  void applyColourToWren(const WbRgb &color);
  void applySkyBoxToWren();

  bool isFirstInstance() { return cBackgroundList.first() == this; }
  // make this the WbBackground instance in use
  void activate();

  // user accessible fields
  WbMFColor *mSkyColor;
  WbMFString *mUrlFields[6];
  WbSFDouble *mLuminosity;

  // skybox related fields
  WrShaderProgram *mSkyboxShaderProgram;
  WrRenderable *mSkyboxRenderable;
  WrMaterial *mSkyboxMaterial;
  WrTransform *mSkyboxTransform;
  WrStaticMesh *mSkyboxMesh;

  // hdr-clearing quad-related fields
  WrShaderProgram *mHdrClearShaderProgram;
  WrRenderable *mHdrClearRenderable;
  WrMaterial *mHdrClearMaterial;
  WrTransform *mHdrClearTransform;
  WrStaticMesh *mHdrClearMesh;

  WrTextureCubeMap *mCubeMapTexture;
  WrTextureCubeMap *mDiffuseIrradianceCubeTexture;
  WrTextureCubeMap *mSpecularIrradianceCubeTexture;

private slots:
  void updateColor();
  void updateCubemap();
  void updateLuminosity();
};

#endif
