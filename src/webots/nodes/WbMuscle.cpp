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

#include "WbMuscle.hpp"

#include "WbAppearance.hpp"
#include "WbFieldChecker.hpp"
#include "WbHingeJoint.hpp"
#include "WbJoint.hpp"
#include "WbMFColor.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMotor.hpp"
#include "WbNodeUtilities.hpp"
#include "WbPreferences.hpp"
#include "WbSliderJoint.hpp"
#include "WbSolid.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"
#include "WbWrenVertexArrayFrameListener.hpp"

#include <cmath>

#include <QtCore/QFileInfo>
#include <QtGui/QImage>

#include <wren/dynamic_mesh.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/transform.h>

const int SUBDIVISION = 16;            // ellipsoid subdivision
WbVector2 *gCircleCoordinates = NULL;  // based on SUBDIVISION value

void WbMuscle::init() {
  mVolume = findSFDouble("volume");
  mStartOffset = findSFVector3("startOffset");
  mEndOffset = findSFVector3("endOffset");
  mColors = findMFColor("color");
  mCastShadows = findSFBool("castShadows");
  mVisible = findSFBool("visible");

  // deprecated field
  mMaxRadius = findSFDouble("maxRadius");
  if (mVolume->value() == 0.01 && mMaxRadius->value() != 0.0)
    parsingWarn(tr("'maxRadius' field is deprecated, please use the 'volume' field instead."));

  mEndPoint = NULL;
  mHeight = 0.1;
  mPreviousHeight = 0.1;
  mRadius = 0.1;
  mStatus = 0.0;
  mMaterialStatus = mStatus;
  mDirectionInverted = false;
  mParentPose = NULL;
  mMatrix = WbMatrix4();

  // WREN
  mTransform = NULL;
  mRenderable = NULL;
  mMaterial = NULL;
  mMesh = NULL;
  mTexture = NULL;
  mQImage = NULL;

  // precompute circle coordinates
  if (gCircleCoordinates == NULL) {
    gCircleCoordinates = new WbVector2[SUBDIVISION + 1];
    const double da = 2 * M_PI / SUBDIVISION;
    double angle = 0.0;
    for (int i = 0; i < SUBDIVISION; ++i, angle += da)
      gCircleCoordinates[i] = WbVector2(cos(angle), sin(angle));
    gCircleCoordinates[SUBDIVISION] = gCircleCoordinates[0];
  }
}

WbMuscle::WbMuscle(WbTokenizer *tokenizer) : WbBaseNode("Muscle", tokenizer) {
  init();
}

WbMuscle::WbMuscle(const WbMuscle &other) : WbBaseNode(other) {
  init();
}

WbMuscle::WbMuscle(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbMuscle::~WbMuscle() {
  if (isPostFinalizedCalled())
    WbWrenVertexArrayFrameListener::instance()->unsubscribeMuscle(this);

  if (areWrenObjectsInitialized()) {
    wr_material_delete(mMaterial);
    wr_material_delete(wr_renderable_get_material(mRenderable, "picking"));

    wr_node_delete(WR_NODE(mRenderable));
    wr_node_delete(WR_NODE(mTransform));

    wr_dynamic_mesh_delete(mMesh);
    wr_texture_delete(WR_TEXTURE(mTexture));
  }
  delete mQImage;
}

void WbMuscle::postFinalize() {
  WbBaseNode::postFinalize();

  mParentPose = WbNodeUtilities::findUpperPose(this);
  const WbJoint *joint = dynamic_cast<WbJoint *>(parentNode()->parentNode());
  updateEndPoint(joint->solidEndPoint());
  WbWrenVertexArrayFrameListener::instance()->subscribeMuscle(this);
  updateMaterial();

  connect(joint, &WbJoint::endPointChanged, this, &WbMuscle::updateEndPoint);
  connect(mVolume, &WbSFDouble::changed, this, &WbMuscle::updateVolume);
  connect(mStartOffset, &WbSFVector3::changed, this, &WbMuscle::computeStretchedDimensions);
  connect(mEndOffset, &WbSFVector3::changed, this, &WbMuscle::computeStretchedDimensions);
  connect(mColors, &WbMFColor::changed, this, &WbMuscle::updateMaterial);
  connect(mCastShadows, &WbSFBool::changed, this, &WbMuscle::updateCastShadows);
  connect(mVisible, &WbSFBool::changed, this, &WbMuscle::updateVisible);
}

void WbMuscle::updateVolume() {
  if (WbFieldChecker::resetDoubleIfNonPositive(this, mVolume, 0.2))
    return;

  // field check
  if (areWrenObjectsInitialized())
    computeStretchedDimensions();
}

void WbMuscle::updateMaterial() {
  if (!areWrenObjectsInitialized())
    return;

  if (mColors->isEmpty())
    return;

  WbRgb rgb(mColors->item(0));
  WbVector3 c(rgb.red(), rgb.green(), rgb.blue());
  int index = (0.0 == mStatus) ? 0 : (mStatus < 0.0 ? 1 : 2);
  if (mDirectionInverted && index > 0)
    // invert contracting and relaxing colors
    index = 2 - index + 1;
  if (index >= mColors->size())
    index = mColors->size() - 1;
  if (index != 0) {
    rgb = mColors->item(index);
    const double percent = abs(mStatus);
    const double oneMinusPercent = 1.0f - percent;
    c[0] = c[0] * oneMinusPercent + rgb.red() * percent;
    c[1] = c[1] * oneMinusPercent + rgb.green() * percent;
    c[2] = c[2] * oneMinusPercent + rgb.blue() * percent;
  }
  float ambient[3];
  float diffuse[3];
  c.toFloatArray(diffuse);
  c *= 0.2;
  c.toFloatArray(ambient);
  wr_phong_material_set_diffuse(mMaterial, diffuse);
  wr_phong_material_set_ambient(mMaterial, ambient);
}

void WbMuscle::updateCastShadows() {
  if (areWrenObjectsInitialized())
    wr_renderable_set_cast_shadows(mRenderable, mCastShadows->value());
}

void WbMuscle::updateVisible() {
  if (areWrenObjectsInitialized()) {
    updateVisibility();
    computeStretchedDimensions();
  }
}

void WbMuscle::updateVisibility() const {
  wr_node_set_visible(WR_NODE(mTransform), mVisible->value());
}

void WbMuscle::updateEndPointPosition() {
  updateEndPoint(const_cast<WbSolid *>(mEndPoint));
}

void WbMuscle::updateEndPoint(WbBaseNode *node) {
  const WbSolid *solid = dynamic_cast<WbSolid *>(node);
  const WbJoint *joint = dynamic_cast<WbJoint *>(parentNode()->parentNode());
  if (mEndPoint != solid) {
    if (mEndPoint) {
      disconnect(mEndPoint->translationFieldValue(), &WbSFVector3::changedByOde, this, &WbMuscle::stretch);
      disconnect(joint, &WbJoint::updateMuscleStretch, this, &WbMuscle::stretch);
      disconnect(mEndPoint, &WbSolid::translationOrRotationChangedByUser, this, &WbMuscle::updateEndPointPosition);
    }
    mEndPoint = solid;
    if (mEndPoint) {
      // listen to solid endPoint position change
      connect(mEndPoint->translationFieldValue(), &WbSFVector3::changedByOde, this, &WbMuscle::stretch, Qt::UniqueConnection);
      // listen to joint force percentage
      connect(joint, &WbJoint::updateMuscleStretch, this, &WbMuscle::updateStretchForce, Qt::UniqueConnection);
      // listen to solid endPoint artificial translation change
      connect(mEndPoint, &WbSolid::translationOrRotationChangedByUser, this, &WbMuscle::updateEndPointPosition,
              Qt::UniqueConnection);
    }
  }

  computeStretchedDimensions();
}

void WbMuscle::updateStretchForce(double forcePercentage, bool immediateUpdate, int motorIndex) {
  const WbMotor *motor = dynamic_cast<WbMotor *>(parentNode());
  if (motor->positionIndex() != motorIndex)
    return;
  mStatus = forcePercentage;
  if (mStatus > 1.0)
    mStatus = 1.0;
  if (mStatus < -1.0)
    mStatus = -1.0;
  if (mStatus > 0.0)  // relaxing
    mDirectionInverted = mHeight > mPreviousHeight;
  else if (mStatus < 0.0)  // contracting
    mDirectionInverted = mHeight < mPreviousHeight;

  if (immediateUpdate)
    computeStretchedDimensions();
}

void WbMuscle::stretch() {
  computeStretchedDimensions();
}

void WbMuscle::computeStretchedDimensions() {
  if (!mEndPoint || !mVisible->value())
    return;

  const WbVector3 t =
    mEndPoint->rotation().toMatrix3() * mEndOffset->value() + mEndPoint->translation() - mStartOffset->value();
  mPreviousHeight = mHeight;
  mHeight = t.length();
  // keep spheroid volume constant: V = 4 / 3 * M_PI * height * radius^2
  mRadius = sqrt((3.0 * mVolume->value()) / (4.0 * M_PI * mHeight));

  // compute mesh matrix
  mMatrix.setIdentity();
  mMatrix.setTranslation(mStartOffset->value());
  const WbVector3 y(0, 1, 0);
  double dotProduct = y.dot(t) / mHeight;
  if (fabs(1 - dotProduct) > 1e-06) {
    WbVector3 w = y.cross(t);
    w.normalize();
    double angle = WbMathsUtilities::clampedAcos(dotProduct);
    assert(!std::isnan(angle));
    mMatrix.setRotation(w.x(), w.y(), w.z(), angle);
  }

  if (areWrenObjectsInitialized())
    WbWrenVertexArrayFrameListener::instance()->subscribeMuscle(this);
}

void WbMuscle::createWrenObjects() {
  WbBaseNode::createWrenObjects();

  // Load muscle texture (512x512 ARGB)
  QByteArray imagePath = QFileInfo("gl:textures/muscle.png").absoluteFilePath().toUtf8();
  mTexture = wr_texture_2d_copy_from_cache(imagePath.constData());
  if (!mTexture) {
    mQImage = new QImage(imagePath.constData());
    assert(!mQImage->isNull());

    mTexture = wr_texture_2d_new();
    wr_texture_set_size(WR_TEXTURE(mTexture), mQImage->width(), mQImage->height());
    wr_texture_2d_set_data(mTexture, reinterpret_cast<const char *>(mQImage->bits()));
    wr_texture_2d_set_file_path(mTexture, imagePath);
    wr_texture_setup(WR_TEXTURE(mTexture));
  }

  mMesh = wr_dynamic_mesh_new(true, true, false);
  createMeshBuffers();

  mMaterial = wr_phong_material_new();
  // These values were taken from WREN muscle material
  const float diffuseColor[3] = {1.0f, 0.0f, 0.0f};
  const float ambientColor[3] = {0.2f, 0.0f, 0.0f};
  wr_phong_material_set_diffuse(mMaterial, diffuseColor);
  wr_phong_material_set_ambient(mMaterial, ambientColor);
  wr_material_set_texture(mMaterial, WR_TEXTURE(mTexture), 0);
  wr_material_set_default_program(mMaterial, WbWrenShaders::phongShader());
  wr_material_set_stencil_ambient_emissive_program(mMaterial, WbWrenShaders::phongStencilAmbientEmissiveShader());
  wr_material_set_stencil_diffuse_specular_program(mMaterial, WbWrenShaders::phongStencilDiffuseSpecularShader());

  mRenderable = wr_renderable_new();
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VM_REGULAR);
  wr_renderable_set_material(mRenderable, mMaterial, NULL);
  wr_renderable_set_receive_shadows(mRenderable, true);
  wr_renderable_set_cast_shadows(mRenderable, mCastShadows->value());

  mTransform = wr_transform_new();
  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(wrenNode(), WR_NODE(mTransform));
  setWrenNode(mTransform);

  WbWrenPicker::setPickable(mRenderable, uniqueId(), true);
}

void WbMuscle::createMeshBuffers() {
  wr_dynamic_mesh_clear(mMesh);

  // first triangles row
  for (int i = 0; i < SUBDIVISION; ++i) {
    wr_dynamic_mesh_add_index(mMesh, i);
    wr_dynamic_mesh_add_index(mMesh, i + SUBDIVISION);
    wr_dynamic_mesh_add_index(mMesh, i + SUBDIVISION + 1);
  }
  // quads (2 triangles) body
  int vIndex = SUBDIVISION;
  const int bodyRowCount = SUBDIVISION - 2;
  for (int j = 0; j < bodyRowCount; ++j) {
    int thisIndex = vIndex;
    for (int i = 0; i < SUBDIVISION; ++i, ++thisIndex) {
      const int nextRowIndex = thisIndex + SUBDIVISION + 1;
      // quad first triangle
      wr_dynamic_mesh_add_index(mMesh, thisIndex);
      wr_dynamic_mesh_add_index(mMesh, nextRowIndex);
      wr_dynamic_mesh_add_index(mMesh, thisIndex + 1);
      // quad second triangle
      wr_dynamic_mesh_add_index(mMesh, thisIndex + 1);
      wr_dynamic_mesh_add_index(mMesh, nextRowIndex);
      wr_dynamic_mesh_add_index(mMesh, nextRowIndex + 1);
    }
    vIndex += SUBDIVISION + 1;
  }
  // last triangles row
  for (int i = 0; i < SUBDIVISION; ++i, ++vIndex) {
    wr_dynamic_mesh_add_index(mMesh, vIndex + SUBDIVISION + 1);
    wr_dynamic_mesh_add_index(mMesh, vIndex + 1);
    wr_dynamic_mesh_add_index(mMesh, vIndex);
  }

  // set texture coordinates
  const float invSub = 1.0f / SUBDIVISION;
  // first vertices
  float u = 0.5f * invSub;
  for (int i = 0; i < SUBDIVISION; ++i, u += invSub) {
    const float UvCoordinate[2] = {u, 0.0f};
    wr_dynamic_mesh_add_texture_coordinate(mMesh, UvCoordinate);
  }
  // body
  float v = invSub;
  for (int j = 1; j < SUBDIVISION; ++j, v += invSub) {
    u = 0;
    for (int i = 0; i <= SUBDIVISION; ++i, u += invSub) {
      const float UvCoordinate[2] = {u, v};
      wr_dynamic_mesh_add_texture_coordinate(mMesh, UvCoordinate);
    }
  }
  // last vertices
  u = 0.5f * invSub;
  for (int i = 0; i < SUBDIVISION; ++i, u += invSub) {
    const float UvCoordinate[2] = {u, 1.0f};
    wr_dynamic_mesh_add_texture_coordinate(mMesh, UvCoordinate);
  }

  updateMeshCoordinates();
}

void WbMuscle::updateMeshCoordinates() {
  assert(areWrenObjectsInitialized());

  wr_dynamic_mesh_clear_selected(mMesh, true, true, false, false);

  // set vertex coordinates and normals
  const WbMatrix3 rm = mMatrix.extracted3x3Matrix();
  int vIndex = 0;
  int nIndex = 0;
  // top vertices
  for (int i = 0; i < SUBDIVISION; ++i, vIndex += 3, nIndex += 3) {
    float vertex[3];
    (mMatrix * WbVector4(0, 0, 0, 1)).toVector3().toFloatArray(vertex);
    wr_dynamic_mesh_add_vertex(mMesh, vertex);
    float normal[3];
    (rm * WbVector3(0, -1, 0)).toFloatArray(normal);
    wr_dynamic_mesh_add_normal(mMesh, normal);
  }
  // body
  const double h2 = mHeight * 0.5;
  const double dy = mHeight / SUBDIVISION;
  const double normalFactor = h2 / mRadius;
  double y = dy - h2;
  for (int j = 1; j < SUBDIVISION; ++j, y += dy) {
    const double d = y / h2;
    const double r = mRadius * sqrt(1 - d * d);
    for (int i = 0; i <= SUBDIVISION; ++i, vIndex += 3, nIndex += 3) {
      const WbVector4 coord = WbVector4(r * gCircleCoordinates[i].x(), y + h2, r * gCircleCoordinates[i].y(), 1.0);

      float vertex[3];
      (mMatrix * coord).toVector3().toFloatArray(vertex);
      wr_dynamic_mesh_add_vertex(mMesh, vertex);
      float normal[3];
      (rm * WbVector3(gCircleCoordinates[i].x(), coord.y() * normalFactor, gCircleCoordinates[i].y())).toFloatArray(normal);
      wr_dynamic_mesh_add_normal(mMesh, normal);
    }
  }
  // bottom vertices
  for (int i = 0; i < SUBDIVISION; ++i, vIndex += 3, nIndex += 3) {
    float vertex[3];
    (mMatrix * WbVector4(0, mHeight, 0, 1)).toVector3().toFloatArray(vertex);
    wr_dynamic_mesh_add_vertex(mMesh, vertex);
    float normal[3];
    (rm * WbVector3(0, 1, 0)).toFloatArray(normal);
    wr_dynamic_mesh_add_normal(mMesh, normal);
  }
}

void WbMuscle::animateMesh() {
  if (mStatus != mMaterialStatus) {
    mMaterialStatus = mStatus;
    updateMaterial();
  }

  updateMeshCoordinates();
}
