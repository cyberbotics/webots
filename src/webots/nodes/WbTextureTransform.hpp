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

#ifndef WB_TEXTURE_TRANSFORM_HPP
#define WB_TEXTURE_TRANSFORM_HPP

#include "WbBaseNode.hpp"
#include "WbSFVector2.hpp"

class WbVector2;

struct WrMaterial;
struct WrTextureTransform;

class WbTextureTransform : public WbBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbTextureTransform(WbTokenizer *tokenizer = NULL);
  WbTextureTransform(const WbTextureTransform &other);
  explicit WbTextureTransform(const WbNode &other);
  virtual ~WbTextureTransform();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TEXTURE_TRANSFORM; }
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  void modifyWrenMaterial(WrMaterial *wrenMaterial);
  WbVector2 transformUVCoordinate(const WbVector2 &uv) const;

  // animation functions
  void translate(const WbVector2 &offset);
  WbVector2 translation() { return mTranslation->value(); }
  void setTranslation(WbVector2 translation) { mTranslation->setValue(translation); }

  // export
  QStringList fieldsToSynchronizeWithX3D() const override;

signals:
  void changed();

private:
  // user accessible fields
  WbSFVector2 *mCenter;
  WbSFDouble *mRotation;
  WbSFVector2 *mScale;
  WbSFVector2 *mTranslation;

  WrTextureTransform *mWrenTextureTransform;

  WbTextureTransform &operator=(const WbTextureTransform &);  // non copyable
  WbNode *clone() const override { return new WbTextureTransform(*this); }
  void init();
  void destroyWrenObjects();

private slots:
  void updateCenter();
  void updateRotation();
  void updateScale();
  void updateTranslation();
};

#endif
