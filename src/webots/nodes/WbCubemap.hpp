// Copyright 1996-2018 Cyberbotics Ltd.
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

#ifndef WB_CUBEMAP_HPP
#define WB_CUBEMAP_HPP

#include "WbBaseNode.hpp"

struct WrTextureCubeMap;
struct WrMaterial;

class WbCubemap : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbCubemap(WbTokenizer *tokenizer = NULL);
  WbCubemap(const WbCubemap &other);
  explicit WbCubemap(const WbNode &other);
  virtual ~WbCubemap();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CUBEMAP; }
  void postFinalize() override;

  void loadWrenTexture();
  void clearWrenTexture();

  void modifyWrenMaterial(WrMaterial *material);

  bool isValid() const { return mIsValid; }
  bool isEquirectangular() const { return mIsEquirectangular; }

  QString textureUrls(int index) const { return mTextureUrls[index]; }

  WrTextureCubeMap *skyboxMap() const { return mDefaultCubeTexture; }
  WrTextureCubeMap *diffuseIrradianceMap() const { return mDiffuseIrradianceCubeTexture; }
  WrTextureCubeMap *specularIrradianceMap() const { return mSpecularIrradianceCubeTexture; }

  static const QString *textureSuffixes();

signals:
  void changed();
  void bakeCompleted();
  void cubeTexturesDestroyed();

private:
  WbCubemap &operator=(const WbCubemap &);  // non copyable
  // reimplemented functions
  WbNode *clone() const override { return new WbCubemap(*this); }

  // misc functions
  void init();

  // user accessible fields
  WbSFString *mTextureBaseName;
  WbSFString *mDirectory;

  QString mTextureUrls[6];

  QImage *mQImages[6];

  // skybox related fields
  WrTextureCubeMap *mDefaultCubeTexture;
  WrTextureCubeMap *mDiffuseIrradianceCubeTexture;
  WrTextureCubeMap *mSpecularIrradianceCubeTexture;
  bool mIsValid;
  bool mIsEquirectangular;

private slots:
  void updateWrenTexture();
};

#endif
