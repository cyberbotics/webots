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

#ifndef WB_IMAGE_TEXTURE_HPP
#define WB_IMAGE_TEXTURE_HPP

#include "WbBaseNode.hpp"
#include "WbVector2.hpp"

#include <QtCore/QSet>

#include <assimp/material.h>

class WbRgb;
class WbDownloader;

class QImage;
class QIODevice;

struct WrMaterial;
struct WrTexture;
struct WrTextureTransform;

class WbImageTexture : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbImageTexture(WbTokenizer *tokenizer = NULL);
  WbImageTexture(const WbImageTexture &other);
  explicit WbImageTexture(const WbNode &other);
  WbImageTexture(const aiMaterial *material, aiTextureType textureType, const QString &parentPath);
  virtual ~WbImageTexture();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_IMAGE_TEXTURE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  void pickColor(const WbVector2 &uv, WbRgb &pickedColor);

  // WREN
  virtual const WrTexture *wrenTexture() const { return mWrenTexture; }
  void modifyWrenMaterial(WrMaterial *wrenMaterial, const int mainTextureIndex, const int backgroundTextureIndex);

  // Texture features
  int width() const;
  int height() const;

  // external texture
  void setExternalTexture(WrTexture *texture, unsigned char *image, double ratioX, double ratioY);
  void removeExternalTexture();
  void setBackgroundTexture(WrTexture *backgroundTexture);
  void unsetBackgroundTexture();

  const QString path() const;

  void setRole(const QString &role) { mRole = role; }

  void exportShallowNode(const WbWriter &writer) const;

  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void changed();

protected:
  bool exportNodeHeader(WbWriter &writer) const override;
  void exportNodeFields(WbWriter &writer) const override;

private:
  // user accessible fields
  WbMFString *mUrl;
  WbSFBool *mRepeatS;
  WbSFBool *mRepeatT;
  WbSFInt *mFiltering;

  // Wren
  WrTexture *mWrenTexture;
  WrTexture *mWrenBackgroundTexture;
  WrTextureTransform *mWrenTextureTransform;
  int mWrenTextureIndex;

  // The following attributes are used when a texture is coming from an external source (e.g Display device)
  bool mExternalTexture;
  WbVector2 mExternalTextureRatio;
  const unsigned char *mExternalTextureData;

  QImage *mImage;
  int mUsedFiltering;
  bool mIsMainTextureTransparent;
  QString mRole;  // Role in a PBR appearance.
  WbDownloader *mDownloader;

  WbImageTexture &operator=(const WbImageTexture &);  // non copyable
  WbNode *clone() const override { return new WbImageTexture(*this); }
  void init();
  void initFields();
  void updateWrenTexture();
  void applyTextureParams();
  void destroyWrenTexture();
  bool loadTexture();
  bool loadTextureData(QIODevice *device);

private slots:
  void updateUrl();
  void updateRepeatS();
  void updateRepeatT();
  void updateFiltering();
  void downloadUpdate();
};

#endif
