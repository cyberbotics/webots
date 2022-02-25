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
#include "WbSFString.hpp"

class WbBoundingSphere;
class WbSFString;
class WbDownloader;
struct WrTransform;
struct WrStaticMesh;
struct WrRenderable;
struct WrMaterial;

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
  void preFinalize() override;
  void postFinalize() override;

private slots:
  void updateUrl();

private:
  WbColladaShape &operator=(const WbColladaShape &);  // non copyable
  WbNode *clone() const override { return new WbColladaShape(*this); }
  void init();

  // Ray tracing
  mutable WbBoundingSphere *mBoundingSphere;

  WbSFString *mUrl;

  QVector<WrRenderable *> mRenderables;
  QVector<WrMaterial *> mMaterials;
  QVector<WrStaticMesh *> mMeshes;

  QString colladaPath() const;

  void updateShape();
  void createWrenMeshes();
  void deleteWrenMeshes();
};

#endif
