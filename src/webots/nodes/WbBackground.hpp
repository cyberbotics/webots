// Copyright 1996-2022 Cyberbotics Ltd.
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

class WbDownloader;
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
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;

  // accessor
  WbRgb skyColor() const;
  double luminosity() const { return mLuminosity->value(); }

  WrTextureCubeMap *irradianceCubeTexture() { return mIrradianceCubeTexture; };

signals:
  void cubemapChanged();
  void luminosityChanged();

protected:
  void exportNodeFields(WbWriter &writer) const override;

private:
  static QList<WbBackground *> cBackgroundList;

  // reimplemented functions
  WbNode *clone() const override { return new WbBackground(*this); }

  // misc functions
  WbBackground &operator=(const WbBackground &);  // non copyable
  void init();
  void destroySkyBox();
  void applyColorToWren(const WbRgb &color);
  bool loadTexture(int i);
  bool loadIrradianceTexture(int i);
  void applySkyBoxToWren();

  bool isFirstInstance() { return cBackgroundList.first() == this; }
  // make this the WbBackground instance in use
  void activate();
  void downloadAsset(const QString &url, int index, bool postpone);

  // user accessible fields
  WbMFColor *mSkyColor;
  WbMFString *mUrlFields[6];
  WbMFString *mIrradianceUrlFields[6];
  WbSFDouble *mLuminosity;

  // texture loading fields
  QImage *mTexture[6];
  bool mTextureHasAlpha;
  int mTextureSize;
  float *mIrradianceTexture[6];
  int mIrradianceWidth;
  int mIrradianceHeight;
  int mUrlCount;

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
  WrTextureCubeMap *mIrradianceCubeTexture;

  WbDownloader *mDownloader[12];

private slots:
  void updateColor();
  void updateCubemap();
  void updateLuminosity();
  void downloadUpdate();
};

#endif
