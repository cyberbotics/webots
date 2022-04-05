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

#ifndef WB_COLLADA_SHAPE_HPP
#define WB_COLLADA_SHAPE_HPP

#include "WbBaseNode.hpp"

#include <assimp/material.h>

class WbBoundingSphere;
class WbMFString;
class WbDownloader;
class WbPbrAppearance;
class WbRgb;

struct WrTransform;
struct WrStaticMesh;
struct WrRenderable;
struct WrMaterial;

struct aiMaterial;

class WbColladaShape : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbColladaShape(WbTokenizer *tokenizer = NULL);
  WbColladaShape(const WbColladaShape &other);
  explicit WbColladaShape(const WbNode &other);
  virtual ~WbColladaShape();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COLLADA_SHAPE; }
  void downloadAssets() override;
  void postFinalize() override;

protected:
  void exportNodeContents(WbVrmlWriter &writer) const override;

private slots:
  void updateUrl();
  void updateCcw();
  void updateCastShadows();
  void updateIsPickable();
  void updateAppearance();

  void downloadUpdate();

private:
  WbColladaShape &operator=(const WbColladaShape &);  // non copyable
  WbNode *clone() const override { return new WbColladaShape(*this); }
  void init();

  WbDownloader *mDownloader;

  // node fields
  WbMFString *mUrl;
  WbSFBool *mCcw;
  WbSFBool *mCastShadows;
  WbSFBool *mIsPickable;

  // wren objects
  QVector<WrRenderable *> mWrenRenderables;
  QVector<WrMaterial *> mWrenMaterials;
  QVector<WrStaticMesh *> mWrenMeshes;
  QVector<WrTransform *> mWrenTransforms;
  QVector<WbPbrAppearance *> mPbrAppearances;

  // segmentation and rangefinder materials
  QVector<WrMaterial *> mWrenSegmentationMaterials;
  QVector<WrMaterial *> mWrenEncodeDepthMaterials;

  const QString vrmlPbrAppearance(const aiMaterial *material);
  bool addTextureMap(QString &vrml, const aiMaterial *material, const QString &mapName, aiTextureType textureType);
  QString colladaPath() const;

  void setSegmentationColor(const WbRgb &color);

  void createWrenObjects() override;
  void deleteWrenObjects();
};

#endif
