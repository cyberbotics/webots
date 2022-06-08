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

#include "WbCadShape.hpp"

#include "WbApplicationInfo.hpp"
#include "WbBackground.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDownloader.hpp"
#include "WbField.hpp"
#include "WbMFString.hpp"
#include "WbNetwork.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPbrAppearance.hpp"
#include "WbRgb.hpp"
#include "WbSolid.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

void WbCadShape::init() {
  mUrl = findMFString("url");
  mCcw = findSFBool("ccw");
  mCastShadows = findSFBool("castShadows");
  mIsPickable = findSFBool("isPickable");

  mWrenRenderables.clear();
  mWrenMaterials.clear();
  mWrenMeshes.clear();
  mWrenTransforms.clear();
  mPbrAppearances.clear();

  mWrenSegmentationMaterials.clear();
  mWrenEncodeDepthMaterials.clear();

  mObjMaterials.clear();
  mMaterialDownloaders.clear();

  mDownloader = NULL;
  mBoundingSphere = NULL;
}

WbCadShape::WbCadShape(WbTokenizer *tokenizer) : WbBaseNode("CadShape", tokenizer) {
  init();
}

WbCadShape::WbCadShape(const WbCadShape &other) : WbBaseNode(other) {
  init();
}

WbCadShape::WbCadShape(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbCadShape::~WbCadShape() {
  if (areWrenObjectsInitialized())
    deleteWrenObjects();

  delete mBoundingSphere;
}

void WbCadShape::downloadAssets() {
  if (mUrl->size() == 0)
    return;

  const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
  if (!WbUrl::isWeb(completeUrl) || (WbNetwork::instance()->isCached(completeUrl) && areMaterialAssetsAvailable(completeUrl)))
    return;

  if (mDownloader != NULL && mDownloader->hasFinished())
    delete mDownloader;

  mDownloader = new WbDownloader(this);
  if (!WbWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
    connect(mDownloader, &WbDownloader::complete, this, &WbCadShape::downloadUpdate);

  mDownloader->download(QUrl(completeUrl));
}

void WbCadShape::downloadUpdate() {
  updateUrl();
  WbWorld::instance()->viewpoint()->emit refreshRequired();
}

void WbCadShape::retrieveMaterials() {
  const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);

  qDeleteAll(mMaterialDownloaders);
  mMaterialDownloaders.clear();

  QStringList rawMaterials = objMaterialList(completeUrl);
  foreach (QString material, rawMaterials) {
    const QString newUrl = generateMaterialUrl(material, completeUrl);
    mObjMaterials.insert(material, newUrl);
    // prepare a downloader
    WbDownloader *downloader = new WbDownloader();
    connect(downloader, &WbDownloader::complete, this, &WbCadShape::materialDownloadTracker);
    mMaterialDownloaders.push_back(downloader);
  }

  // start all downloads only when the vector is entirely populated (to avoid racing conditions)
  assert(mMaterialDownloaders.size() == mObjMaterials.size());
  QMapIterator<QString, QString> it(mObjMaterials);
  int i = 0;
  while (it.hasNext()) {
    it.next();
    mMaterialDownloaders[i++]->download(QUrl(it.value()));
  }
}

QString WbCadShape::generateMaterialUrl(const QString &material, const QString &completeUrl) {
  QString materialUrl = material;
  // manufacture material url from url of the obj file
  materialUrl.replace("\\", "/");  // use cross-platform forward slashes
  if (materialUrl.startsWith("./"))
    materialUrl.remove(0, 2);

  QString prefixUrl = QUrl(completeUrl).adjusted(QUrl::RemoveFilename).toString();
  while (materialUrl.startsWith("../")) {
    prefixUrl = prefixUrl.left(prefixUrl.lastIndexOf("/"));
    materialUrl.remove(0, 3);
  }

  return prefixUrl + materialUrl;
}

void WbCadShape::materialDownloadTracker() {
  bool finished = true;
  foreach (WbDownloader *downloader, mMaterialDownloaders) {
    if (!downloader->hasFinished())
      finished = false;

    if (!downloader->error().isEmpty()) {
      warn(downloader->error());  // failure downloading or file does not exist (404)
      return;
    }
  }

  if (finished)
    updateUrl();
}

void WbCadShape::postFinalize() {
  WbBaseNode::postFinalize();

  connect(mUrl, &WbMFString::changed, this, &WbCadShape::updateUrl);
  connect(mCcw, &WbSFBool::changed, this, &WbCadShape::updateCcw);
  connect(mCastShadows, &WbSFBool::changed, this, &WbCadShape::updateCastShadows);
  connect(mIsPickable, &WbSFBool::changed, this, &WbCadShape::updateIsPickable);

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::backgroundColorChanged, this,
          &WbCadShape::createWrenObjects);

  mBoundingSphere = new WbBoundingSphere(this);

  updateUrl();
  updateCcw();
  updateCastShadows();
  updateIsPickable();

  // apply segmentation color
  const WbSolid *solid = WbNodeUtilities::findUpperSolid(this);
  WbRgb color(0.0, 0.0, 0.0);
  while (solid) {
    if (solid->recognitionColorSize() > 0) {
      color = solid->recognitionColor(0);
      break;
    }
    solid = WbNodeUtilities::findUpperSolid(solid);
  }
  setSegmentationColor(color);
}

void WbCadShape::updateUrl() {
  if (cadPath().isEmpty()) {
    deleteWrenObjects();
    return;
  }

  // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
  const int n = mUrl->size();
  for (int i = 0; i < n; i++) {
    QString item = mUrl->item(i);
    mUrl->blockSignals(true);
    mUrl->setItem(i, item.replace("\\", "/"));
    mUrl->blockSignals(false);
  }

  if (n > 0) {
    const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
    if (WbUrl::isWeb(completeUrl)) {
      if (mDownloader && !mDownloader->error().isEmpty()) {
        warn(mDownloader->error());  // failure downloading or file does not exist (404)
        deleteWrenObjects();
        delete mDownloader;
        mDownloader = NULL;
        return;
      }

      if (!WbNetwork::instance()->isCached(completeUrl)) {
        if (mDownloader && mDownloader->hasFinished()) {
          delete mDownloader;
          mDownloader = NULL;
        }

        downloadAssets();  // url was changed from the scene tree or supervisor
        return;
      }
    }

    const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();
    if (extension == "obj" && WbUrl::isWeb(completeUrl)) {
      // ensure any mtl referenced by the obj file are also downloaded
      if (areMaterialAssetsAvailable(completeUrl)) {
        mObjMaterials.clear();
        // generate mapping between referenced files and cached files
        QStringList rawMaterials = objMaterialList(completeUrl);
        foreach (QString material, rawMaterials) {
          QString adjustedUrl = generateMaterialUrl(material, completeUrl);
          assert(WbNetwork::instance()->isCached(adjustedUrl));
          if (!mObjMaterials.contains(material))
            mObjMaterials.insert(material, adjustedUrl);
        }
      } else {
        retrieveMaterials();
        return;
      }
    }

    createWrenObjects();
  }
}

bool WbCadShape::areMaterialAssetsAvailable(const QString &url) {
  QStringList rawMaterials = objMaterialList(url);  // note: 'dae' files will generate an empty list
  foreach (QString material, rawMaterials) {
    if (!WbNetwork::instance()->isCached(generateMaterialUrl(material, url)))
      return false;
  }
  return true;
}

QStringList WbCadShape::objMaterialList(const QString &url) const {
  const QString extension = url.mid(url.lastIndexOf('.') + 1).toLower();
  if (extension != "obj")
    return QStringList();

  QStringList materials;
  QFile objFile;
  if (WbNetwork::instance()->isCached(url))
    objFile.setFileName(WbNetwork::instance()->get(url));
  else  // local file
    objFile.setFileName(url);
  if (objFile.open(QIODevice::ReadOnly)) {
    QString content = QString(objFile.readAll());
    content = content.replace("\r\n", "\n");

    QStringList lines = content.split('\n', Qt::SkipEmptyParts);
    foreach (QString line, lines) {
      QString cleanLine = line.trimmed();
      if (!cleanLine.startsWith("mtllib"))
        continue;

      cleanLine = cleanLine.replace("mtllib ", "");
      materials << cleanLine.split(".mtl", Qt::SkipEmptyParts);
      for (int i = 0; i < materials.size(); i++)
        materials[i] += ".mtl";
    }
  } else
    warn(tr("File '%1' cannot be read.").arg(url));

  objFile.close();

  return materials;
}

void WbCadShape::updateCcw() {
  for (WrRenderable *renderable : mWrenRenderables)
    wr_renderable_invert_front_face(renderable, !mCcw->value());
}

void WbCadShape::updateCastShadows() {
  for (WrRenderable *renderable : mWrenRenderables)
    wr_renderable_set_cast_shadows(renderable, mCastShadows->value());
}

void WbCadShape::updateIsPickable() {
  for (WrRenderable *renderable : mWrenRenderables)
    WbWrenPicker::setPickable(renderable, uniqueId(), mIsPickable->value());
}

void WbCadShape::setSegmentationColor(const WbRgb &color) {
  const float segmentationColor[3] = {(float)color.red(), (float)color.green(), (float)color.blue()};
  for (WrMaterial *segmentationMaterial : mWrenSegmentationMaterials)
    wr_phong_material_set_linear_diffuse(segmentationMaterial, segmentationColor);
}

void WbCadShape::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  deleteWrenObjects();

  if (mUrl->size() == 0)
    return;

  const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
  const QString extension = completeUrl.mid(completeUrl.lastIndexOf('.') + 1).toLower();

  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_CAMERAS | aiComponent_LIGHTS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS);

  unsigned int flags = aiProcess_ValidateDataStructure | aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                       aiProcess_JoinIdenticalVertices | aiProcess_OptimizeGraph | aiProcess_RemoveComponent |
                       aiProcess_FlipUVs;

  const aiScene *scene;
  if (extension != "dae" && extension != "obj") {
    warn(tr("Invalid url '%1'. CadShape node expects file in Collada ('.dae') or Wavefront ('.obj') format.").arg(completeUrl));
    return;
  }

  if (WbUrl::isWeb(completeUrl)) {
    if (!WbNetwork::instance()->isCached(completeUrl)) {
      if (mDownloader == NULL)  // never attempted to download it, try now
        downloadAssets();
      return;
    }

    QFile file(WbNetwork::instance()->get(completeUrl));
    if (!file.open(QIODevice::ReadOnly)) {
      warn(tr("File could not be read: '%1'").arg(completeUrl));
      return;
    }

    QByteArray data = file.readAll();
    // for remote 'obj' files that reference materials, this reference needs to be changed to point to the cached asset instead
    if (extension == "obj") {
      QMapIterator<QString, QString> it(mObjMaterials);
      while (it.hasNext()) {
        it.next();
        data.replace(it.key().toUtf8(), WbNetwork::instance()->get(it.value()).toUtf8());
      }
    }

    scene = importer.ReadFileFromMemory(data.constData(), data.size(), flags, extension.toUtf8().constData());
  } else
    scene = importer.ReadFile(completeUrl.toStdString().c_str(), flags);

  if (!scene) {
    warn(tr("Invalid data, please verify mesh file: %1").arg(importer.GetErrorString()));
    return;
  }

  // Assimp fix for up_axis, adapted from https://github.com/assimp/assimp/issues/849
  if (extension == "dae")  // rotate around X by 90° to swap Y and Z axis
    scene->mRootNode->mTransformation =
      aiMatrix4x4(1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1) * scene->mRootNode->mTransformation;

  std::list<aiNode *> queue;
  queue.push_back(scene->mRootNode);

  aiNode *node;
  while (!queue.empty()) {
    node = queue.front();
    queue.pop_front();

    for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
      const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];

      // compute absolute transform of this node from all the parents
      const int vertices = mesh->mNumVertices;
      const int faces = mesh->mNumFaces;
      if (vertices < 3)  // silently ignore meshes with less than 3 vertices as they are invalid
        continue;

      if (vertices > 100000)
        warn(tr("Mesh '%1' has more than 100'000 vertices, it is recommended to reduce the number of vertices.")
               .arg(mesh->mName.C_Str()));

      aiMatrix4x4 transform;
      aiNode *current = node;
      while (current != NULL) {
        transform *= current->mTransformation;
        current = current->mParent;
      }

      // create the arrays
      int currentCoordIndex = 0;
      float *const coordData = new float[3 * vertices];
      int currentNormalIndex = 0;
      float *const normalData = new float[3 * vertices];
      int currentTexCoordIndex = 0;
      float *const texCoordData = new float[2 * vertices];
      int currentIndexIndex = 0;
      unsigned int *const indexData = new unsigned int[3 * faces];

      for (size_t j = 0; j < mesh->mNumVertices; ++j) {
        // extract the coordinate
        const aiVector3D vertice = transform * mesh->mVertices[j];
        coordData[currentCoordIndex++] = vertice[0];
        coordData[currentCoordIndex++] = vertice[1];
        coordData[currentCoordIndex++] = vertice[2];
        // extract the normal
        const aiVector3D normal = transform * mesh->mNormals[j];
        normalData[currentNormalIndex++] = normal[0];
        normalData[currentNormalIndex++] = normal[1];
        normalData[currentNormalIndex++] = normal[2];
        // extract the texture coordinate
        if (mesh->HasTextureCoords(0)) {
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].x;
          texCoordData[currentTexCoordIndex++] = mesh->mTextureCoords[0][j].y;
        } else {
          texCoordData[currentTexCoordIndex++] = 0.5;
          texCoordData[currentTexCoordIndex++] = 0.5;
        }
      }

      // create the index array
      for (size_t j = 0; j < mesh->mNumFaces; ++j) {
        const aiFace face = mesh->mFaces[j];
        if (face.mNumIndices < 3)  // we want to skip lines
          continue;
        assert(face.mNumIndices == 3);
        indexData[currentIndexIndex++] = face.mIndices[0];
        indexData[currentIndexIndex++] = face.mIndices[1];
        indexData[currentIndexIndex++] = face.mIndices[2];
      }

      if (currentIndexIndex == 0)  // if all faces turned out to be invalid, ignore the mesh
        continue;

      WrStaticMesh *staticMesh =
        wr_static_mesh_new(vertices, currentIndexIndex, coordData, normalData, texCoordData, texCoordData, indexData, false);

      mWrenMeshes.push_back(staticMesh);

      delete[] coordData;
      delete[] normalData;
      delete[] texCoordData;
      delete[] indexData;

      // retrieve material properties
      const aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];

      // determine how image textures referenced in the collada/wavefront file will be searched for
      QString fileRoot = mUrl->item(0);
      fileRoot = fileRoot.replace("\\", "/");               // use cross-platform forward slashes
      fileRoot = fileRoot.left(fileRoot.lastIndexOf("/"));  // do not include the final forward slash

      // init from assimp material
      WbPbrAppearance *pbrAppearance = new WbPbrAppearance(material, fileRoot);
      pbrAppearance->preFinalize();
      pbrAppearance->postFinalize();
      connect(pbrAppearance, &WbPbrAppearance::changed, this, &WbCadShape::updateAppearance);

      WrMaterial *wrenMaterial = wr_pbr_material_new();
      pbrAppearance->modifyWrenMaterial(wrenMaterial);

      mPbrAppearances.push_back(pbrAppearance);
      mWrenMaterials.push_back(wrenMaterial);
    }
  }

  for (int i = 0; i < mWrenMeshes.size(); ++i) {
    WrRenderable *renderable = wr_renderable_new();
    wr_renderable_set_material(renderable, mWrenMaterials[i], NULL);
    wr_renderable_set_mesh(renderable, WR_MESH(mWrenMeshes[i]));
    wr_renderable_set_receive_shadows(renderable, true);
    wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VM_REGULAR);
    wr_renderable_set_cast_shadows(renderable, mCastShadows->value());
    wr_renderable_invert_front_face(renderable, !mCcw->value());
    WbWrenPicker::setPickable(renderable, uniqueId(), mIsPickable->value());

    // set material for range finder camera rendering
    WrMaterial *depthMaterial = wr_phong_material_new();
    wr_material_set_default_program(depthMaterial, WbWrenShaders::encodeDepthShader());
    wr_renderable_set_material(renderable, depthMaterial, "encodeDepth");

    // set material for segmentation camera rendering
    WrMaterial *segmentationMaterial = wr_phong_material_new();
    wr_material_set_default_program(segmentationMaterial, WbWrenShaders::segmentationShader());
    wr_renderable_set_material(renderable, segmentationMaterial, "segmentation");

    WrTransform *transform = wr_transform_new();
    wr_transform_attach_child(wrenNode(), WR_NODE(transform));
    setWrenNode(transform);
    wr_transform_attach_child(transform, WR_NODE(renderable));
    wr_node_set_visible(WR_NODE(transform), true);

    mWrenRenderables.push_back(renderable);
    mWrenTransforms.push_back(transform);
    mWrenEncodeDepthMaterials.push_back(depthMaterial);
    mWrenSegmentationMaterials.push_back(segmentationMaterial);
  }

  if (mBoundingSphere)
    recomputeBoundingSphere();
}

void WbCadShape::updateAppearance() {
  assert(mPbrAppearances.size() == mWrenMaterials.size());

  for (int i = 0; i < mPbrAppearances.size(); ++i)
    mPbrAppearances[i]->modifyWrenMaterial(mWrenMaterials[i]);
}

void WbCadShape::deleteWrenObjects() {
  for (WrRenderable *renderable : mWrenRenderables) {
    wr_material_delete(wr_renderable_get_material(renderable, "picking"));
    wr_node_delete(WR_NODE(renderable));
  }

  for (WrStaticMesh *mesh : mWrenMeshes)
    wr_static_mesh_delete(mesh);

  for (WrMaterial *material : mWrenMaterials)
    wr_material_delete(material);

  for (WrMaterial *depthMaterial : mWrenEncodeDepthMaterials)
    wr_material_delete(depthMaterial);

  for (WrMaterial *segmentationMaterial : mWrenSegmentationMaterials)
    wr_material_delete(segmentationMaterial);

  for (WbPbrAppearance *appearance : mPbrAppearances)
    delete appearance;

  for (WrTransform *transform : mWrenTransforms)
    wr_node_delete(WR_NODE(transform));

  mWrenRenderables.clear();
  mWrenMeshes.clear();
  mWrenMaterials.clear();
  mWrenEncodeDepthMaterials.clear();
  mWrenSegmentationMaterials.clear();
  mWrenTransforms.clear();

  mPbrAppearances.clear();
}

void WbCadShape::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  const WbVector3 &scale = absoluteScale();
  for (WrStaticMesh *mesh : mWrenMeshes) {
    float sphere[4];
    wr_static_mesh_get_bounding_sphere(mesh, sphere);

    const WbVector3 center(sphere[0], sphere[1], sphere[2]);
    double radius = sphere[3];
    radius = radius / std::max(std::max(scale.x(), scale.y()), scale.z());
    const WbBoundingSphere meshBoundingSphere(NULL, center, radius);
    mBoundingSphere->enclose(&meshBoundingSphere);
  }
}

const WbVector3 WbCadShape::absoluteScale() const {
  const WbTransform *const ut = upperTransform();
  return ut ? ut->absoluteScale() : WbVector3(1.0, 1.0, 1.0);
}

void WbCadShape::exportNodeFields(WbWriter &writer) const {
  WbBaseNode::exportNodeFields(writer);

  if (!writer.isX3d())
    return;

  if (mUrl->size() == 0)
    return;

  WbField urlFieldCopy(*findField("url", true));
  for (int i = 0; i < mUrl->size(); ++i) {
    if (WbUrl::isLocalUrl(mUrl->value()[i])) {
      QString newUrl = mUrl->value()[i];
      dynamic_cast<WbMFString *>(urlFieldCopy.value())
        ->setItem(i, newUrl.replace("webots://", "https://raw.githubusercontent.com/" + WbApplicationInfo::repo() + "/" +
                                                   WbApplicationInfo::branch() + "/"));
    } else if (WbUrl::isWeb(mUrl->value()[i]))
      continue;
    else {
      const QString meshPath(WbUrl::computePath(this, "url", mUrl, i));
      if (writer.isWritingToFile()) {
        QString newUrl = WbUrl::exportMesh(this, mUrl, i, writer);
        dynamic_cast<WbMFString *>(urlFieldCopy.value())->setItem(i, newUrl);
      }

      const QString &url(mUrl->item(i));
      writer.addResourceToList(url, meshPath);
    }
  }
  const QString completeUrl = WbUrl::computePath(this, "url", mUrl->item(0), false);
  const QString prefix = completeUrl.left(completeUrl.lastIndexOf('/'));
  for (QString material : objMaterialList(completeUrl)) {
    QString newUrl;
    if (writer.isWritingToFile())
      newUrl = WbUrl::exportResource(this, material, WbUrl::computePath(this, "url", mUrl, 0), writer.relativeMeshesPath(),
                                     writer, false);
    else
      newUrl = prefix + '/' + material;

    dynamic_cast<WbMFString *>(urlFieldCopy.value())->addItem(newUrl);
    writer.addResourceToList(newUrl, newUrl);
  }

  for (int i = 0; i < mPbrAppearances.size(); ++i)
    mPbrAppearances[i]->exportShallowNode(writer);

  urlFieldCopy.write(writer);

  findField("ccw", true)->write(writer);
  findField("castShadows", true)->write(writer);
  findField("isPickable", true)->write(writer);
}

QString WbCadShape::cadPath() const {
  return WbUrl::computePath(this, "url", mUrl, false);
}
