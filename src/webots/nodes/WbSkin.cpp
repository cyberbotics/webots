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

#include "WbSkin.hpp"

#include "WbAbstractAppearance.hpp"
#include "WbAppearance.hpp"
#include "WbBasicJoint.hpp"
#include "WbBoundingSphere.hpp"
#include "WbDataStream.hpp"
#include "WbDownloader.hpp"
#include "WbMFNode.hpp"
#include "WbNetwork.hpp"
#include "WbNodeUtilities.hpp"
#include "WbProject.hpp"
#include "WbProtoModel.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSFRotation.hpp"
#include "WbSFVector3.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbUrl.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/dynamic_mesh.h>
#include <wren/file_import.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/skeleton.h>
#include <wren/skeleton_bone.h>
#include <wren/transform.h>

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <QtCore/QQueue>

void WbSkin::init() {
  mSkeleton = NULL;
  mSkeletonTransform = NULL;
  mRenderablesTransform = NULL;
  mBonesWarningPrinted = false;
  mBoneOrientationRequest = NULL;
  mBonePositionRequest = NULL;
  mNeedConfigureAfterModelChanged = false;
  mIsModelUrlValid = false;
  mDownloader = NULL;

  mBoneMesh = NULL;
  mBoneMaterial = NULL;

  mBoundingSphere = NULL;

  mName = findSFString("name");
  mModelUrl = findSFString("modelUrl");
  mAppearanceField = findMFNode("appearance");
  mBonesField = findMFNode("bones");
  mCastShadows = findSFBool("castShadows");
}

WbSkin::WbSkin(WbTokenizer *tokenizer) : WbBaseNode("Skin", tokenizer), WbAbstractTransform(this), WbDevice() {
  init();
}

WbSkin::WbSkin(const WbSkin &other) : WbBaseNode(other), WbAbstractTransform(this), WbDevice(other) {
  init();
}

WbSkin::WbSkin(const WbNode &other) : WbBaseNode(other), WbAbstractTransform(this), WbDevice() {
  init();
}

WbSkin::~WbSkin() {
  if (areWrenObjectsInitialized()) {
    deleteWrenSkeleton();

    wr_material_delete(mBoneMaterial);
    wr_static_mesh_delete(mBoneMesh);
  }

  delete mBoundingSphere;
}

void WbSkin::downloadAssets() {
  WbBaseNode::downloadAssets();
  WbMFIterator<WbMFNode, WbNode *> it(mAppearanceField);
  while (it.hasNext()) {
    WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(it.next());
    assert(appearance);
    appearance->downloadAssets();
  }

  const QString &url = mModelUrl->value();
  if (!url.isEmpty()) {
    const QString &completeUrl = WbUrl::computePath(this, "modelUrl", url);
    if (WbUrl::isWeb(completeUrl)) {
      delete mDownloader;
      mDownloader = new WbDownloader(this);
      if (!WbWorld::instance()->isLoading())  // URL changed from the scene tree or supervisor
        connect(mDownloader, &WbDownloader::complete, this, &WbSkin::downloadUpdate);

      mDownloader->download(QUrl(completeUrl));
    }
  }
}

void WbSkin::downloadUpdate() {
  updateModel();
  WbWorld::instance()->viewpoint()->emit refreshRequired();
}

void WbSkin::preFinalize() {
  WbBaseNode::preFinalize();

  WbMFIterator<WbMFNode, WbNode *> it(mBonesField);
  while (it.hasNext())
    dynamic_cast<WbSolidReference *>(it.next())->preFinalize();

  it = WbMFIterator<WbMFNode, WbNode *>(mAppearanceField);
  while (it.hasNext()) {
    WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(it.next());
    assert(appearance);
    appearance->preFinalize();
  }

  WbAbstractTransform::checkScale();
}

void WbSkin::postFinalize() {
  WbBaseNode::postFinalize();

  WbMFIterator<WbMFNode, WbNode *> it(mBonesField);
  while (it.hasNext())
    dynamic_cast<WbSolidReference *>(it.next())->postFinalize();
  it = WbMFIterator<WbMFNode, WbNode *>(mAppearanceField);
  while (it.hasNext()) {
    WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(it.next());
    assert(appearance);
    appearance->postFinalize();
  }

  connect(mTranslation, &WbSFVector3::changed, this, &WbSkin::updateTranslation);
  connect(mRotation, &WbSFRotation::changed, this, &WbSkin::updateRotation);
  connect(mScale, SIGNAL(changed()), this, SLOT(updateScale()));
  connect(mModelUrl, &WbSFString::changed, this, &WbSkin::updateModelUrl);
  connect(mAppearanceField, &WbMFNode::changed, this, &WbSkin::updateAppearance, Qt::QueuedConnection);
  connect(mBonesField, &WbMFNode::changed, this, &WbSkin::updateBones);
  connect(mCastShadows, &WbSFBool::changed, this, &WbSkin::updateCastShadows);

  mBoundingSphere = new WbBoundingSphere(this);
  updateModelUrl();

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

void WbSkin::updateTranslation() {
  WbAbstractTransform::updateTranslation();
  if (mSkeleton && mBonesField->size() > 0)
    wr_skeleton_update_offset(mSkeleton);
}

void WbSkin::updateRotation() {
  WbAbstractTransform::updateRotation();
  if (mSkeleton && mBonesField->size() > 0)
    wr_skeleton_update_offset(mSkeleton);
}

void WbSkin::updateScale(bool warning) {
  WbAbstractTransform::updateScale(warning);
  if (mSkeleton && mBonesField->size() > 0)
    wr_skeleton_update_offset(mSkeleton);
}

void WbSkin::applyToScale() {
  WbAbstractTransform::applyToScale();
}

int WbSkin::constraintType() const {
  static const int CONSTRAINT = WbWrenAbstractResizeManipulator::NO_CONSTRAINT;
  return CONSTRAINT;
}

void WbSkin::showResizeManipulator(bool enabled) {
  if (isProtoInstance()) {
    WbBaseNode::showResizeManipulator(enabled);
    return;
  }

  WbAbstractTransform::showResizeManipulator(enabled);

  emit visibleHandlesChanged(enabled);
}

QString WbSkin::modelPath() const {
  if (mModelUrl->value().isEmpty())
    return QString();
  return WbUrl::computePath(this, "modelUrl", mModelUrl->value(), true);
}

void WbSkin::setSegmentationColor(const WbRgb &color) {
  const float segmentationColor[3] = {(float)color.red(), (float)color.green(), (float)color.blue()};

  for (int i = 0; i < mSegmentationMaterials.size(); ++i) {
    if (!mSegmentationMaterials[i])
      continue;
    wr_phong_material_set_linear_diffuse(mSegmentationMaterials[i], segmentationColor);
  }
}

void WbSkin::updateModelUrl() {
  if (!isPostFinalizedCalled())
    return;

  if (!mDownloader)
    mIsModelUrlValid = false;

  if (!mModelUrl->value().isEmpty()) {
    // we want to replace the windows backslash path separators (if any) with cross-platform forward slashes
    QString value = mModelUrl->value();
    mModelUrl->blockSignals(true);
    mModelUrl->setValue(value.replace("\\", "/"));
    mModelUrl->blockSignals(false);

    const QFileInfo modelInfo(mModelUrl->value());
    const QStringList supportedExtensions = {"FBX"};
    if (!supportedExtensions.contains(modelInfo.completeSuffix(), Qt::CaseInsensitive)) {
      warn(tr("Invalid modelUrl '%1'. Supported formats are: '%2'.")
             .arg(mModelUrl->value())
             .arg(supportedExtensions.join("', '")));
      return;
    }

    const QString &completeUrl = WbUrl::computePath(this, "modelUrl", mModelUrl->value());
    if (!WbWorld::instance()->isLoading() && WbUrl::isWeb(completeUrl) &&
        !WbNetwork::instance()->isCachedWithMapUpdate(completeUrl)) {
      // URL was changed from the scene tree or supervisor
      downloadAssets();
      mIsModelUrlValid = true;
      return;
    }
  }

  mIsModelUrlValid = true;
  updateModel();
}

void WbSkin::updateAppearance() {
  if (!mSkeleton)
    return;

  WbAbstractAppearance *appearance = NULL;
  QStringList undefinedList(mMaterialNames);
  WbMFIterator<WbMFNode, WbNode *> it(mAppearanceField);
  while (it.hasNext()) {
    int index = -1;
    QString appearanceName;
    bool duplicated = false;
    appearance = dynamic_cast<WbAbstractAppearance *>(it.next());
    assert(appearance);
    if (appearance) {
      connect(appearance, &WbAbstractAppearance::nameChanged, this, &WbSkin::updateAppearanceName, Qt::UniqueConnection);
      appearanceName = appearance->name();
      index = mMaterialNames.indexOf(appearanceName);
      if (index >= 0 && undefinedList.contains(appearanceName)) {
        mMaterials[index] = appearance->modifyWrenMaterial(mMaterials[index]);
        connect(appearance, &WbAbstractAppearance::changed, this, &WbSkin::updateMaterial, Qt::UniqueConnection);
      } else
        duplicated = true;

      undefinedList.removeAll(appearanceName);
    }

    if (index < 0)
      parsingWarn(tr("No material named '%1' could be found in mesh file.").arg(appearanceName));
    else if (!duplicated)
      wr_renderable_set_material(mRenderables[index], mMaterials[index], NULL);
    else
      parsingWarn(tr("Duplicated Appearance with name '%1' in 'appearance' field. "
                     "Only the first instance will be used.")
                    .arg(appearanceName));
  }

  for (int i = 0; i < undefinedList.size(); ++i) {
    const int index = mMaterialNames.indexOf(undefinedList[i]);
    mMaterials[index] = WbAppearance::fillWrenDefaultMaterial(mMaterials[index]);
    parsingWarn(tr("Undefined Appearance with name '%1'.").arg(undefinedList[i]));
    wr_renderable_set_material(mRenderables[index], mMaterials[index], NULL);
  }
}

void WbSkin::updateMaterial() {
  WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(sender());
  assert(appearance);
  if (appearance) {
    const int materialIndex = mMaterialNames.indexOf(appearance->name());
    if (materialIndex >= 0 && appearance->areWrenObjectsInitialized())
      mMaterials[materialIndex] = appearance->modifyWrenMaterial(mMaterials[materialIndex]);
    else
      mMaterials[materialIndex] = WbAppearance::fillWrenDefaultMaterial(mMaterials[materialIndex]);

    wr_renderable_set_material(mRenderables[materialIndex], mMaterials[materialIndex], NULL);
  }
}

void WbSkin::updateAppearanceName(const QString &newName, const QString &prevName) {
  const bool newNameDefined = mMaterialNames.contains(newName);
  const bool prevNameDefined = mMaterialNames.contains(prevName);
  if (!newNameDefined && !prevNameDefined)
    return;  // nothing to do

  // look for other definition of prevName and newName materials
  WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(sender());
  WbAbstractAppearance *other = NULL;
  bool isAfter = false;
  bool newNameExistsBefore = false;
  bool prevNameExistsBefore = false;
  bool newNameExistsAfter = false;
  WbMFIterator<WbMFNode, WbNode *> it(mAppearanceField);
  while (it.hasNext()) {
    WbAbstractAppearance *node = dynamic_cast<WbAbstractAppearance *>(it.next());
    if (node == appearance) {
      isAfter = true;
      continue;
    }

    if (isAfter) {
      if (!prevNameExistsBefore && other == NULL && node->name() == prevName)
        other = node;
      newNameExistsAfter |= node->name() == newName;
      if (other != NULL && newNameExistsAfter)
        return;
    } else {
      newNameExistsBefore |= node->name() == newName;
      prevNameExistsBefore |= node->name() == prevName;
    }
  }

  if (newNameExistsBefore || newNameExistsAfter)
    // another definition of newName material is specified before this one
    parsingWarn(tr("Duplicated Appearance with name '%1' in 'appearance' field. "
                   "Only the first instance will be used.")
                  .arg(newName));

  if (newNameDefined && !newNameExistsBefore) {
    // modify newName material
    // this appearance is the first one defining the newName material
    const int index = mMaterialNames.indexOf(newName);
    WrMaterial *material = mMaterials[index];
    mMaterials[index] = appearance->modifyWrenMaterial(material);
    wr_renderable_set_material(mRenderables[index], mMaterials[index], NULL);
    connect(appearance, &WbAbstractAppearance::changed, this, &WbSkin::updateMaterial, Qt::UniqueConnection);
  }

  if (prevNameDefined && !prevNameExistsBefore) {
    // modify prevName material
    // this appearance was the first one defining the prevName material
    const int index = mMaterialNames.indexOf(prevName);
    WrMaterial *material = mMaterials[index];

    if (other == NULL) {
      material = WbAppearance::fillWrenDefaultMaterial(material);
      parsingWarn(tr("Undefined Appearance with name '%1'.").arg(newName));
    } else
      material = other->modifyWrenMaterial(material);

    mMaterials[index] = material;
    wr_renderable_set_material(mRenderables[index], material, NULL);
  }
}

void WbSkin::updateBones() {
  WbMFIterator<WbMFNode, WbNode *> it(mBonesField);
  while (it.hasNext()) {
    WbSolidReference *ref = dynamic_cast<WbSolidReference *>(it.next());
    assert(ref);
    if (ref) {
      ref->updateName();
      connect(ref, &WbSolidReference::changed, this, &WbSkin::updateBones, Qt::UniqueConnection);
    }
  }

  if (isPostFinalizedCalled())
    updateModel();
}

void WbSkin::updateCastShadows() {
  for (WrRenderable *renderable : mRenderables)
    wr_renderable_set_cast_shadows(renderable, mCastShadows->value());
}

void WbSkin::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  WbMFIterator<WbMFNode, WbNode *> it(mAppearanceField);
  while (it.hasNext()) {
    WbAbstractAppearance *appearance = dynamic_cast<WbAbstractAppearance *>(it.next());
    assert(appearance);
    if (appearance)
      appearance->createWrenObjects();
  }

  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::optionalRenderingChanged, this,
          &WbSkin::updateOptionalRendering);

  // Create bone mesh
  const float vertices[72] = {
    0.0f, 0.0f,  0.0f,  -0.1f, 0.1f,  0.25f, 0.0f,  0.0f,  0.0f,  -0.1f, -0.1f, 0.25f, 0.0f,  0.0f, 0.0f,  0.1f,  -0.1f, 0.25f,
    0.0f, 0.0f,  0.0f,  0.1f,  0.1f,  0.25f, 0.0f,  0.0f,  1.0f,  -0.1f, 0.1f,  0.25f, 0.0f,  0.0f, 1.0f,  -0.1f, -0.1f, 0.25f,
    0.0f, 0.0f,  1.0f,  0.1f,  -0.1f, 0.25f, 0.0f,  0.0f,  1.0f,  0.1f,  0.1f,  0.25f, 0.1f,  0.1f, 0.25f, 0.1f,  -0.1f, 0.25f,
    0.1f, -0.1f, 0.25f, -0.1f, -0.1f, 0.25f, -0.1f, -0.1f, 0.25f, -0.1f, 0.1f,  0.25f, -0.1f, 0.1f, 0.25f, 0.1f,  0.1f,  0.25f};

  mBoneMesh = wr_static_mesh_line_set_new(24, vertices, NULL);

  mBoneMaterial = wr_phong_material_new();
  wr_material_set_default_program(mBoneMaterial, WbWrenShaders::lineSetShader());
  const float color[3] = {1.0f, 1.0f, 1.0f};
  wr_phong_material_set_color(mBoneMaterial, color);

  mSkeletonTransform = wr_transform_new();
  wr_transform_attach_child(wrenNode(), WR_NODE(mSkeletonTransform));
  setWrenNode(mSkeletonTransform);

  if (mBonesField->size() > 0)
    updateBones();

  updateModel();
}

void WbSkin::reset(const QString &id) {
  WbBaseNode::reset(id);

  for (int i = 0; i < mAppearanceField->size(); ++i)
    mAppearanceField->item(i)->reset(id);
  if (mBonesField->size() > 0) {
    for (int i = 0; i < mBonesField->size(); ++i)
      mBonesField->item(i)->reset(id);
  } else if (mSkeleton) {
    // reset mesh skeleton
    const int bonesCount = wr_skeleton_get_bone_count(mSkeleton);
    for (int i = 0; i < bonesCount; ++i) {
      const WbVector3 &p = mInitialSkeletonPosition.at(i);
      setBonePosition(i, p.x(), p.y(), p.z(), false);
      const WbRotation &o = mInitialSkeletonOrientation.at(i);
      setBoneOrientation(i, o.x(), o.y(), o.z(), o.angle(), false);
    }
  }
}

void WbSkin::updateModel() {
  applyTranslationToWren();
  applyRotationToWren();
  applyScaleToWren();

  createWrenSkeleton();
  if (mSkeleton) {
    updateAppearance();
    wr_skeleton_apply_binding_pose(mSkeleton);
  }
}

void WbSkin::createWrenSkeleton() {
  deleteWrenSkeleton();

  if (!mIsModelUrlValid || mModelUrl->value().isEmpty())
    return;

  const QString meshFilePath(modelPath());
  if (meshFilePath.isEmpty())
    return;

  WrDynamicMesh **meshes = NULL;
  const char **materialNames = NULL;
  int count;
  const char *error;
  if (WbUrl::isWeb(meshFilePath)) {
    if (WbNetwork::instance()->isCachedWithMapUpdate(meshFilePath)) {
      QFile file(WbNetwork::instance()->get(meshFilePath));
      if (!file.open(QIODevice::ReadOnly))
        return;
      const QByteArray &data = file.readAll();
      const char *hint = meshFilePath.mid(meshFilePath.lastIndexOf('.') + 1).toUtf8().constData();
      error = wr_import_skeleton_from_memory(data.constData(), data.size(), hint, &mSkeleton, &meshes, &materialNames, &count);
    } else
      return;
  } else
    error = wr_import_skeleton_from_file(meshFilePath.toStdString().c_str(), &mSkeleton, &meshes, &materialNames, &count);

  if (error) {
    parsingWarn(tr("Unable to read mesh file '%1': %2").arg(meshFilePath).arg(error));
    return;
  }

  if (mDownloader != NULL)
    delete mDownloader;
  mDownloader = NULL;

  mRenderablesTransform = wr_transform_new();
  for (int i = 0; i < count; ++i) {
    WrRenderable *renderable = wr_renderable_new();
    WrMaterial *material = WbAppearance::fillWrenDefaultMaterial(NULL);
    wr_renderable_set_material(renderable, material, NULL);
    wr_renderable_set_mesh(renderable, WR_MESH(meshes[i]));
    wr_renderable_set_receive_shadows(renderable, true);
    wr_renderable_set_cast_shadows(renderable, mCastShadows->value());
    wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VM_REGULAR);

    // used for rendering range finder camera
    WrMaterial *depthMaterial = wr_phong_material_new();
    wr_material_set_default_program(depthMaterial, WbWrenShaders::encodeDepthShader());
    wr_renderable_set_material(renderable, depthMaterial, "encodeDepth");

    // used for rendering segmentation camera
    WrMaterial *segmentationMaterial = wr_phong_material_new();
    wr_material_set_default_program(segmentationMaterial, WbWrenShaders::segmentationShader());
    wr_renderable_set_material(renderable, segmentationMaterial, "segmentation");

    wr_transform_attach_child(mRenderablesTransform, WR_NODE(renderable));

    mMaterials.push_back(material);
    mSegmentationMaterials.push_back(segmentationMaterial);
    mEncodeDepthMaterials.push_back(depthMaterial);
    mMeshes.push_back(meshes[i]);
    mMaterialNames.push_back(QString(materialNames[i]));
    mRenderables.push_back(renderable);
    WbWrenPicker::setPickable(renderable, uniqueId(), true);
  }
  delete[] materialNames;
  delete[] meshes;

  // Create skeleton from Webots joints
  const bool webotsBones = mBonesField->size() > 0;
  if (webotsBones) {
    if (!createSkeletonFromWebotsNodes()) {
      deleteWrenSkeleton();
      return;
    }
    wr_transform_attach_child(wrenNode(), WR_NODE(mSkeleton));
    wr_transform_attach_child(wr_scene_get_root(wr_scene_get_instance()), WR_NODE(mRenderablesTransform));
  } else {
    wr_transform_attach_child(wrenNode(), WR_NODE(mRenderablesTransform));

    const bool visible =
      WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_SKIN_SKELETON);
    // Create skeleton representation (special handling for pure visual skeleton)
    for (int i = 0; i < wr_skeleton_get_bone_count(mSkeleton); ++i) {
      WrTransform *bone = WR_TRANSFORM(wr_skeleton_get_bone_by_index(mSkeleton, i));
      WrTransform *copy = wr_transform_copy(bone);
      wr_node_set_visible(WR_NODE(copy), visible);
      mBoneTransforms.push_back(copy);
      mBonesMap[bone] = copy;
    }

    for (int i = 0; i < wr_skeleton_get_bone_count(mSkeleton); ++i) {
      WrSkeletonBone *bone = wr_skeleton_get_bone_by_index(mSkeleton, i);
      WrTransform *parent = wr_node_get_parent(WR_NODE(bone));
      float position[3];
      wr_skeleton_bone_get_position(bone, false, position);
      if (parent) {
        // Attach bone representation
        const float length = WbVector3(position).length();
        const float scale[3] = {length, length, length};
        const float orientation[4] = {M_PI_2, -1, 0, 0};

        WrTransform *boneTransform = createBoneRepresentation(scale, orientation, visible);
        wr_transform_attach_child(mBonesMap[parent], WR_NODE(boneTransform));
        wr_transform_attach_child(mBonesMap[parent], WR_NODE(mBonesMap[WR_TRANSFORM(bone)]));
      } else
        wr_transform_attach_child(wrenNode(), WR_NODE(mBonesMap[WR_TRANSFORM(bone)]));

      float orientation[4];
      wr_skeleton_bone_get_orientation(bone, false, orientation);
      mInitialSkeletonOrientation.append(WbRotation(orientation[1], orientation[2], orientation[3], orientation[0]));
      mInitialSkeletonPosition.append(WbVector3(position));
    }
  }

  if (isPostFinalizedCalled())
    mNeedConfigureAfterModelChanged = true;

  if (mBoundingSphere)
    recomputeBoundingSphere();
}

WrTransform *WbSkin::createBoneRepresentation(const float *scale, const float *orientation, bool visible) {
  WrTransform *boneTransform = wr_transform_new();
  wr_transform_set_scale(boneTransform, scale);
  wr_transform_set_orientation(boneTransform, orientation);
  wr_node_set_visible(WR_NODE(boneTransform), visible);
  mBoneTransforms.push_back(boneTransform);

  WrRenderable *boneRenderable = wr_renderable_new();
  wr_renderable_set_material(boneRenderable, mBoneMaterial, NULL);
  wr_renderable_set_mesh(boneRenderable, WR_MESH(mBoneMesh));
  wr_renderable_set_cast_shadows(boneRenderable, false);
  wr_renderable_set_drawing_mode(boneRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_drawing_order(boneRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_0);
  wr_renderable_set_visibility_flags(boneRenderable, WbWrenRenderingContext::VF_SKIN_SKELETON);
  mRenderables.push_back(boneRenderable);

  wr_transform_attach_child(boneTransform, WR_NODE(boneRenderable));
  return boneTransform;
}

void WbSkin::deleteWrenSkeleton() {
  for (WrRenderable *renderable : mRenderables) {
    // Delete picking material
    wr_material_delete(wr_renderable_get_material(renderable, "picking"));
    wr_node_delete(WR_NODE(renderable));
  }

  for (WrDynamicMesh *mesh : mMeshes)
    wr_dynamic_mesh_delete(mesh);

  for (WrMaterial *material : mMaterials)
    wr_material_delete(material);

  // delete encode depth material
  for (WrMaterial *depthMaterial : mEncodeDepthMaterials)
    wr_material_delete(depthMaterial);

  // delete camera segmentation material
  for (WrMaterial *segmentationMaterial : mSegmentationMaterials)
    wr_material_delete(segmentationMaterial);

  for (WrTransform *transform : mBoneTransforms)
    wr_node_delete(WR_NODE(transform));

  wr_node_delete(WR_NODE(mSkeleton));
  wr_node_delete(WR_NODE(mRenderablesTransform));

  mSkeleton = NULL;
  mRenderablesTransform = NULL;
  mMaterialNames.clear();
  mRenderables.clear();
  mMaterials.clear();
  mSegmentationMaterials.clear();
  mEncodeDepthMaterials.clear();
  mMeshes.clear();
  mBoneTransforms.clear();
  mBonesMap.clear();
  mInitialSkeletonPosition.clear();
  mInitialSkeletonOrientation.clear();
}

bool WbSkin::createSkeletonFromWebotsNodes() {
  // create bones
  QVector<QString> boneNames;
  QVector<WrTransform *> parentBoneList;  // used to detect tail bones
  QMap<WrTransform *, WbSolid *> boneToSolidMap;
  int validBoneCount = 0;
  for (int i = 0; i < mBonesField->size(); ++i) {
    WbSolidReference *ref = dynamic_cast<WbSolidReference *>(mBonesField->item(i));
    if (ref->solid() == NULL)
      continue;

    WbSolid *solid = ref->solid();
    QString boneName = solid->name();
    if (boneNames.contains(boneName)) {
      parsingWarn(tr("Invalid item %1 in 'bones' field: "
                     "duplicated reference to Solid with name '%2'.")
                    .arg(i)
                    .arg(solid->name()));
      continue;
    }

    WrTransform *wrenBone = WR_TRANSFORM(wr_skeleton_get_bone_by_name(mSkeleton, boneName.toStdString().c_str()));
    if (!wrenBone) {
      parsingWarn(tr("Bone named '%1' could not be found in skeleton file.").arg(boneName));
      continue;
    }

    boneToSolidMap[wrenBone] = solid;
    ++validBoneCount;
    boneNames.push_back(boneName);
    parentBoneList.push_back(wr_node_get_parent(WR_NODE(wrenBone)));
  }

  const int skeletonBoneCount = wr_skeleton_get_bone_count(mSkeleton);
  if (validBoneCount != skeletonBoneCount) {
    parsingWarn(tr("Number of valid bones defined in 'bones' field does not match with skeleton file. "
                   "Expected: %1, was: %2.")
                  .arg(skeletonBoneCount)
                  .arg(validBoneCount));
    return false;
  }

  const bool visible = WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_SKIN_SKELETON);
  // Once we're sure bones are valid, attach them to solids & create skeleton representation
  QMap<WrTransform *, WbSolid *>::const_iterator it = boneToSolidMap.constBegin();
  while (it != boneToSolidMap.constEnd()) {
    const WrTransform *wrenBone = it.key();
    const WbSolid *solid = it.value();

    WrTransform *parentBone = wr_node_get_parent(WR_NODE(wrenBone));
    WrTransform *parentWrenNode = parentBone && boneToSolidMap[parentBone] ? boneToSolidMap[parentBone]->wrenNode() : NULL;
    if (parentWrenNode) {
      // Attach bone representation
      const WbVector3 &offset = solid->translation();
      const WbVector3 &scale = solid->scale();
      const float length = (offset * scale).length();
      const float boneScale[3] = {length, length, length};

      // compute orientation (default bone representation pointing in z-axis)
      const WbVector3 unit(0, 0, 1);
      const WbVector3 &norm = offset.normalized();
      const WbVector3 &axis = unit.cross(norm).normalized();
      const float boneOrientation[4] = {(float)unit.angle(norm), (float)axis[0], (float)axis[1], (float)axis[2]};

      WrTransform *boneTransform = createBoneRepresentation(boneScale, boneOrientation, visible);
      wr_transform_attach_child(parentWrenNode, WR_NODE(boneTransform));

      if (!parentBoneList.contains(wrenBone)) {
        // display tail bone with same orientation and scale as parent
        boneTransform = createBoneRepresentation(boneScale, boneOrientation, visible);
        wr_transform_attach_child(solid->wrenNode(), WR_NODE(boneTransform));
      }
    }
    wr_transform_attach_child(solid->wrenNode(), WR_NODE(wrenBone));
    ++it;
  }

  return true;
}

void WbSkin::setBoneOrientation(int boneIndex, double ax, double ay, double az, double angle, bool absolute) {
  WrTransform *bone = WR_TRANSFORM(wr_skeleton_get_bone_by_index(mSkeleton, boneIndex));
  assert(bone);
  const float orientation[4] = {static_cast<float>(angle), static_cast<float>(ax), static_cast<float>(ay),
                                static_cast<float>(az)};

  WrTransform *boneRepresentation = mBonesMap[bone];
  const bool skinSkeletonEnabled =
    WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_SKIN_SKELETON);
  if (absolute) {
    wr_transform_set_absolute_orientation(bone, orientation);
    if (skinSkeletonEnabled)
      wr_transform_set_absolute_orientation(boneRepresentation, orientation);
  } else {
    wr_transform_set_orientation(bone, orientation);
    if (skinSkeletonEnabled)
      wr_transform_set_orientation(boneRepresentation, orientation);
  }

  mBoundingSphere->setOwnerSizeChanged();
}

void WbSkin::setBonePosition(int boneIndex, double x, double y, double z, bool absolute) {
  WrTransform *bone = WR_TRANSFORM(wr_skeleton_get_bone_by_index(mSkeleton, boneIndex));
  assert(bone);
  const float position[3] = {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};

  WrTransform *boneRepresentation = mBonesMap[bone];
  const bool skinSkeletonEnabled =
    WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(WbWrenRenderingContext::VF_SKIN_SKELETON);
  if (absolute) {
    wr_transform_set_absolute_position(bone, position);
    if (skinSkeletonEnabled)
      wr_transform_set_absolute_position(boneRepresentation, position);
  } else {
    wr_transform_set_position(bone, position);
    if (skinSkeletonEnabled)
      wr_transform_set_position(boneRepresentation, position);
  }

  mBoundingSphere->setOwnerSizeChanged();
}

void WbSkin::handleMessage(QDataStream &stream) {
  QString functionName;
  unsigned char absolute;
  unsigned short int index = 0;
  bool invalidSkeleton = false;
  bool webotsSkeletonWarning = false;
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_SKIN_GET_BONE_POSITION: {
      functionName = "wb_skin_get_bone_position";
      stream >> index;
      stream >> absolute;
      assert(mBonePositionRequest == NULL);

      if (!mSkeleton)
        invalidSkeleton = true;
      else {
        WrSkeletonBone *bone = wr_skeleton_get_bone_by_index(mSkeleton, index);
        assert(bone);
        float position[3];
        wr_skeleton_bone_get_position(bone, absolute, position);
        mBonePositionRequest = new WbVector3(position[0], position[1], position[2]);
      }

      break;
    }
    case C_SKIN_GET_BONE_ORIENTATION: {
      functionName = "wb_skin_get_bone_orientation";
      stream >> index;
      stream >> absolute;
      assert(mBoneOrientationRequest == NULL);

      if (!mSkeleton)
        invalidSkeleton = true;
      else {
        WrSkeletonBone *bone = wr_skeleton_get_bone_by_index(mSkeleton, index);
        assert(bone);
        float orientation[4];
        wr_skeleton_bone_get_orientation(bone, absolute, orientation);
        mBoneOrientationRequest = new WbRotation(orientation[1], orientation[2], orientation[3], orientation[0]);
      }

      break;
    }
    case C_SKIN_SET_BONE_POSITION: {
      functionName = "wb_skin_set_bone_position";
      double x, y, z;
      stream >> index;
      stream >> x;
      stream >> y;
      stream >> z;
      stream >> absolute;
      if (!mSkeleton)
        invalidSkeleton = true;
      else if (mBonesField->size() > 0)
        webotsSkeletonWarning = true;
      else
        setBonePosition(index, x, y, z, (bool)absolute);
      break;
    }
    case C_SKIN_SET_BONE_ORIENTATION: {
      functionName = "wb_skin_set_bone_orientation";
      double x, y, z, angle;
      stream >> index;
      stream >> x;
      stream >> y;
      stream >> z;
      stream >> angle;
      stream >> absolute;
      if (!mSkeleton)
        invalidSkeleton = true;
      else if (mBonesField->size() > 0)
        webotsSkeletonWarning = true;
      else
        setBoneOrientation(index, x, y, z, angle, (bool)absolute);
      break;
    }
    default:
      assert(false);
      break;
  }

  if (invalidSkeleton)
    parsingWarn(tr("%1 cannot be executed because no valid skeleton is available.").arg(functionName));

  if (webotsSkeletonWarning && !mBonesWarningPrinted) {
    parsingWarn(tr("Skin animation control using %1 is disabled if a Webots "
                   "skeleton is used.\n")
                  .arg(functionName));
    mBonesWarningPrinted = true;
  }
}

void WbSkin::writeAnswer(WbDataStream &stream) {
  if (mNeedConfigureAfterModelChanged) {
    writeConfigure(stream);
    mNeedConfigureAfterModelChanged = false;
  } else if (mBonePositionRequest != NULL) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_SKIN_GET_BONE_POSITION;
    for (int i = 0; i < 3; ++i)
      stream << (double)(*mBonePositionRequest)[i];
    delete mBonePositionRequest;
    mBonePositionRequest = NULL;
  } else if (mBoneOrientationRequest != NULL) {
    stream << (short unsigned int)tag();
    stream << (unsigned char)C_SKIN_GET_BONE_ORIENTATION;
    for (int i = 0; i < 4; ++i)
      stream << (double)(*mBoneOrientationRequest)[i];
    delete mBoneOrientationRequest;
    mBoneOrientationRequest = NULL;
  }
}

void WbSkin::writeConfigure(WbDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  const int boneCount = mSkeleton ? wr_skeleton_get_bone_count(mSkeleton) : 0;
  stream << boneCount;
  for (int i = 0; i < boneCount; ++i) {
    WrSkeletonBone *bone = wr_skeleton_get_bone_by_index(mSkeleton, i);
    const char *name = wr_skeleton_bone_get_name(bone);
    const int length = strlen(name) + 1;
    stream.writeRawData(name, length);
  }
  mNeedConfigureAfterModelChanged = false;
}

void WbSkin::updateOptionalRendering(int option) {
  if (mSkeleton && option == WbWrenRenderingContext::VF_SKIN_SKELETON) {
    const bool visible = WbWrenRenderingContext::instance()->isOptionalRenderingEnabled(option);

    // Update bone positions & orientations (for pure visual skin only)
    if (mBonesField->size() == 0 && visible) {
      for (int i = 0; i < wr_skeleton_get_bone_count(mSkeleton); ++i) {
        const WrSkeletonBone *bone = wr_skeleton_get_bone_by_index(mSkeleton, i);
        float position[3];
        wr_skeleton_bone_get_position(bone, false, position);
        float orientation[4];
        wr_skeleton_bone_get_orientation(bone, false, orientation);
        WrTransform *visualBone = mBonesMap[WR_TRANSFORM(bone)];
        wr_transform_set_position(visualBone, position);
        wr_transform_set_orientation(visualBone, orientation);
      }
    }

    for (WrTransform *bone : mBoneTransforms)
      wr_node_set_visible(WR_NODE(bone), visible);
  }
}

/////////////////
// Ray tracing //
/////////////////

void WbSkin::recomputeBoundingSphere() const {
  assert(mBoundingSphere);
  mBoundingSphere->empty();

  if (!mSkeleton)
    return;

  const bool convertToLocal = mBonesField->size() > 0;
  WbMatrix4 m;
  if (convertToLocal)
    // WREN resturns absolute coordinates
    m = matrix().pseudoInversed();

  int meshCount = 0;
  // get list with format [centerX1, centerY1, centerZ1, radius1, centerX2, .. ]
  // if spheres are world coordinate system
  float *meshBoundingSphereList = wr_skeleton_compute_bounding_spheres(mSkeleton, meshCount);
  int index = 0;
  for (int c = 0; c < meshCount; ++c) {
    WbVector3 center(meshBoundingSphereList[index], meshBoundingSphereList[index + 1], meshBoundingSphereList[index + 2]);
    double radius = meshBoundingSphereList[index + 3];
    if (convertToLocal) {
      const WbVector3 &scale = absoluteScale();
      radius = radius / std::max(std::max(scale.x(), scale.y()), scale.z());
      center = m * center;
    }
    const WbBoundingSphere meshBoundingSphere(NULL, center, radius);
    mBoundingSphere->enclose(&meshBoundingSphere);
    index += 4;
  }
  delete[] meshBoundingSphereList;
}
