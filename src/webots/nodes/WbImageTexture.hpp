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

#ifndef WB_IMAGE_TEXTURE_HPP
#define WB_IMAGE_TEXTURE_HPP

#include "WbBaseNode.hpp"
#include "WbVector2.hpp"

class WbRgb;

class QImage;

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
  virtual ~WbImageTexture();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_IMAGE_TEXTURE; }
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  void pickColor(WbRgb &pickedColor, const WbVector2 &uv) const;

  // WREN
  virtual const WrTexture *wrenTexture() const { return mWrenTexture; }
  void modifyWrenMaterial(WrMaterial *wrenMaterial, const int mainTextureIndex, const int backgroundTextureIndex);

  // Texture features
  int width() const;
  int height() const;
  int filtering() const;

  // external texture
  void setExternalTexture(WrTexture *texture, unsigned char *image, double ratioX, double ratioY);
  void removeExternalTexture();
  void setBackgroundTexture(WrTexture *backgroundTexture);
  void unsetBackgroundTexture();

  QString path();
  void setContainerField(QString &field);

signals:
  void changed();

protected:
  void exportNodeFields(WbVrmlWriter &writer) const override;
  void exportNodeSubNodes(WbVrmlWriter &writer) const override;

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

  QString mContainerField;
  QImage *mImage;
  bool mIsMainTextureTransparent;

  WbImageTexture &operator=(const WbImageTexture &);  // non copyable
  WbNode *clone() const override { return new WbImageTexture(*this); }
  void init();
  void updateWrenTexture();
  void applyTextureParams();
  void destroyWrenTexture();

private slots:
  void updateUrl();
  void updateRepeatS();
  void updateRepeatT();
  void updateFiltering();
};

#endif
