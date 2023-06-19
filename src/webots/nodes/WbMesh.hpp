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

#ifndef WB_MESH_HPP
#define WB_MESH_HPP

#include "WbTriangleMeshGeometry.hpp"

class WbDownloader;
class WbMFString;
struct aiScene;

class WbMesh : public WbTriangleMeshGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbMesh(WbTokenizer *tokenizer = NULL);
  WbMesh(const WbMesh &other);
  explicit WbMesh(const WbNode &other);
  virtual ~WbMesh();

  void updateTriangleMesh(bool issueWarnings = true) override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MESH; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createResizeManipulator() override;
  void rescale(const WbVector3 &scale) override{};

  // WbTriangleMesh management (see WbTriangleMeshCache.hpp)
  uint64_t computeHash() const override;

  QStringList fieldsToSynchronizeWithX3D() const override;

protected:
  void exportNodeFields(WbWriter &writer) const override;

private:
  // user accessible fields
  WbMFString *mUrl;
  WbSFBool *mCcw;
  WbSFString *mName;
  WbSFInt *mMaterialIndex;
  bool mIsCollada;
  WbDownloader *mDownloader;
  bool mBoundingObjectNeedUpdate;

  WbMesh &operator=(const WbMesh &);  // non copyable
  WbNode *clone() const override { return new WbMesh(*this); }
  void init();
  bool checkIfNameExists(const aiScene *scene, const QString &name) const;

private slots:
  void updateUrl();
  void updateCcw();
  void updateName();
  void updateMaterialIndex();
  void downloadUpdate();
};

#endif
