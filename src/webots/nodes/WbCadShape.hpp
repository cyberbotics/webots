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

#ifndef WB_CAD_SHAPE_HPP
#define WB_CAD_SHAPE_HPP

#include "WbBaseNode.hpp"

#include <QtCore/QMap>

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

class WbCadShape : public WbBaseNode {
  Q_OBJECT

public:
  explicit WbCadShape(WbTokenizer *tokenizer = NULL);
  WbCadShape(const WbCadShape &other);
  explicit WbCadShape(const WbNode &other);
  virtual ~WbCadShape();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CAD_SHAPE; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void updateSegmentationColor(const WbRgb &color) override { setSegmentationColor(color); }

  const WbVector3 absoluteScale() const;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  void exportNodeFields(WbWriter &writer) const override;
  WbBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  void recomputeBoundingSphere() const;

private slots:
  void updateUrl();
  void updateCcw();
  void updateCastShadows();
  void updateIsPickable();
  void updateAppearance();

  void downloadUpdate();
  void materialDownloadTracker();

private:
  WbCadShape &operator=(const WbCadShape &);  // non copyable
  WbNode *clone() const override { return new WbCadShape(*this); }
  void init();

  WbDownloader *mDownloader;
  mutable WbBoundingSphere *mBoundingSphere;

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

  // methods and variables to handle obj materials
  QMap<QString, QString> mObjMaterials;  // maps materials as referenced in the obj to their remote counterpart
  QVector<WbDownloader *> mMaterialDownloaders;
  QStringList objMaterialList(const QString &url) const;
  bool areMaterialAssetsAvailable(const QString &url);
  void retrieveMaterials();

  const QString vrmlPbrAppearance(const aiMaterial *material);
  bool addTextureMap(QString &vrml, const aiMaterial *material, const QString &mapName, aiTextureType textureType);
  QString cadPath() const;

  void setSegmentationColor(const WbRgb &color);

  void createWrenObjects() override;
  void deleteWrenObjects();
};

#endif
