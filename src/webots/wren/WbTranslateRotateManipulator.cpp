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

#include "WbTranslateRotateManipulator.hpp"

#include "WbWrenPicker.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/file_import.h>
#include <wren/node.h>
#include <wren/scene.h>

#include <QtCore/QByteArray>
#include <QtCore/QFileInfo>

static const char *FILE_PATH_ARROW = "gl:meshes/arrow.obj";
static const char *FILE_PATH_CIRCULAR_ARROW = "gl:meshes/circular_arrow.obj";

static const float HANDLES_TRANSPARENCY = 0.01f;

const WbVector3 WbTranslateRotateManipulator::STANDARD_COORDINATE_VECTORS[3] = {
  WbVector3(1.0, 0.0, 0.0), WbVector3(0.0, 1.0, 0.0), WbVector3(0.0, 0.0, 1.0)};

WbTranslateRotateManipulator::WbTranslateRotateManipulator(bool isTranslationAvailable, bool isRotationAvailable) :
  WbWrenAbstractManipulator(3),
  mHasRotationHandles(isRotationAvailable),
  mHasTranslationHandles(isTranslationAvailable),
  mActiveRotationHandleMaterial(NULL),
  mRotationLineTransform(NULL),
  mRotationDoubleArrowTransform(NULL) {
  initializeHandlesEntities();
}

void WbTranslateRotateManipulator::initializeHandlesEntities() {
  // Creates handles materials and axes
  if (mHasTranslationHandles || mHasRotationHandles) {
    const float axesColor[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const float axesAmbientColor[3][3] = {{0.9f, 0.0f, 0.0f}, {0.0f, 0.9f, 0.0f}, {0.0f, 0.0f, 0.9f}};
    const float zeroes[3] = {0.0f, 0.0f, 0.0f};

    mHandlesShader = WbWrenShaders::handlesShader();
    mHandlesPickingShader = WbWrenShaders::handlesPickingShader();

    for (int i = 0; i < 3; ++i) {
      WrMaterial *material = wr_phong_material_new();
      wr_material_set_default_program(material, mHandlesShader);
      wr_phong_material_set_color(material, axesColor[i]);
      mHandlesMaterials[i][0] = material;

      material = wr_phong_material_new();
      wr_material_set_default_program(material, mHandlesShader);
      wr_phong_material_set_all_parameters(material, axesAmbientColor[i], axesColor[i], zeroes, zeroes, 0.2f,
                                           HANDLES_TRANSPARENCY);
      mHandlesMaterials[i][1] = material;
    }

    const float axesCoordinates[3][6] = {
      {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

    mAxesTransform = wr_transform_new();
    for (int i = 0; i < 3; ++i) {
      WrRenderable *renderable = wr_renderable_new();
      wr_renderable_set_scene_culling(renderable, false);
      mRenderables.push_back(renderable);

      WrStaticMesh *mesh = wr_static_mesh_line_set_new(2, axesCoordinates[i], NULL);
      mMeshes.push_back(mesh);

      wr_renderable_set_cast_shadows(renderable, false);
      wr_renderable_set_receive_shadows(renderable, false);
      wr_renderable_set_mesh(renderable, WR_MESH(mesh));
      wr_renderable_set_drawing_mode(renderable, WR_RENDERABLE_DRAWING_MODE_LINES);
      wr_renderable_set_drawing_order(renderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
      wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
      wr_renderable_set_material(renderable, mHandlesMaterials[i][0], NULL);

      wr_transform_attach_child(mAxesTransform, WR_NODE(renderable));
    }
    wr_transform_attach_child(mTransform, WR_NODE(mAxesTransform));

    // Creates infinite axes (used when performing transformation)
    const float infiniteAxesCoordinates[3][6] = {{-100.0f, 0.0f, 0.0f, 100.0f, 0.0f, 0.0f},
                                                 {0.0f, -100.0f, 0.0f, 0.0f, 100.0f, 0.0f},
                                                 {0.0f, 0.0f, -100.0f, 0.0f, 0.0f, 100.0f}};

    for (int i = 0; i < 3; ++i) {
      mInfiniteAxesTransforms[i] = wr_transform_new();
      WrRenderable *renderable = wr_renderable_new();
      wr_renderable_set_scene_culling(renderable, false);
      mRenderables.push_back(renderable);

      WrStaticMesh *mesh = wr_static_mesh_line_set_new(2, infiniteAxesCoordinates[i], NULL);
      mMeshes.push_back(mesh);

      wr_renderable_set_cast_shadows(renderable, false);
      wr_renderable_set_receive_shadows(renderable, false);
      wr_renderable_set_mesh(renderable, WR_MESH(mesh));
      wr_renderable_set_drawing_mode(renderable, WR_RENDERABLE_DRAWING_MODE_LINES);
      wr_renderable_set_drawing_order(renderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
      wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
      wr_renderable_set_material(renderable, mHandlesMaterials[i][0], NULL);

      wr_transform_attach_child(mInfiniteAxesTransforms[i], WR_NODE(renderable));
      wr_transform_attach_child(mTransform, WR_NODE(mInfiniteAxesTransforms[i]));
      wr_node_set_visible(WR_NODE(mInfiniteAxesTransforms[i]), false);
    }
  }

  // We add this tiny offset so that the handles are correctly sorted at rendering
  // (translucent objects are sorted by distance with the camera)
  const float offset[3][3] = {{0.0001f, 0.0f, 0.0f}, {0.0f, 0.0001f, 0.0f}, {0.0f, 0.0f, 0.0001f}};

  const float rotation[3][4] = {{1.570796327f, 0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {1.570796327f, 1.0f, 0.0f, 0.0f}};

  if (mHasTranslationHandles) {
    QByteArray objFile = QFileInfo(FILE_PATH_ARROW).absoluteFilePath().toUtf8();
    WrStaticMesh *mesh;
    const bool success = wr_import_static_mesh_from_obj(objFile.constData(), &mesh);
    assert(success);

    if (success) {
      mMeshes.push_back(mesh);
      for (int i = 0; i < 3; ++i) {
        WrRenderable *renderable = wr_renderable_new();
        wr_renderable_set_scene_culling(renderable, false);
        mRenderables.push_back(renderable);

        WbWrenPicker::setPickable(renderable, ((WbWrenPicker::HANDLES_X_AXIS + i) | WbWrenPicker::HANDLES_TRANSLATE), true);
        WrMaterial *pickingMaterial = wr_renderable_get_material(renderable, "picking");
        wr_material_set_default_program(pickingMaterial, mHandlesPickingShader);

        wr_renderable_set_cast_shadows(renderable, false);
        wr_renderable_set_receive_shadows(renderable, false);
        wr_renderable_set_drawing_order(renderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
        wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
        wr_renderable_set_mesh(renderable, WR_MESH(mesh));
        wr_renderable_set_material(renderable, mHandlesMaterials[i][1], NULL);

        WrTransform *transform = wr_transform_new();
        wr_transform_set_orientation(transform, rotation[i]);
        wr_transform_set_position(transform, offset[i]);
        wr_transform_attach_child(transform, WR_NODE(renderable));
        wr_transform_attach_child(mTransform, WR_NODE(transform));
        mTranslationHandlesTransforms[i] = transform;
      }
    } else
      mHasTranslationHandles = false;
  }

  if (mHasRotationHandles) {
    QByteArray objFile = QFileInfo(FILE_PATH_CIRCULAR_ARROW).absoluteFilePath().toUtf8();
    WrStaticMesh *mesh;
    const bool success = wr_import_static_mesh_from_obj(objFile.constData(), &mesh);
    assert(success);

    if (success) {
      mMeshes.push_back(mesh);
      for (int i = 0; i < 3; ++i) {
        WrRenderable *renderable = wr_renderable_new();
        wr_renderable_set_scene_culling(renderable, false);
        mRenderables.push_back(renderable);

        WbWrenPicker::setPickable(renderable, ((WbWrenPicker::HANDLES_X_AXIS + i) | WbWrenPicker::HANDLES_ROTATE), true);
        WrMaterial *pickingMaterial = wr_renderable_get_material(renderable, "picking");
        wr_material_set_default_program(pickingMaterial, mHandlesPickingShader);

        wr_renderable_set_cast_shadows(renderable, false);
        wr_renderable_set_receive_shadows(renderable, false);
        wr_renderable_set_drawing_order(renderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
        wr_renderable_set_visibility_flags(renderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
        wr_renderable_set_mesh(renderable, WR_MESH(mesh));
        wr_renderable_set_material(renderable, mHandlesMaterials[i][1], NULL);

        WrTransform *transform = wr_transform_new();
        wr_transform_set_orientation(transform, rotation[i]);
        wr_transform_set_position(transform, offset[i]);
        wr_transform_attach_child(transform, WR_NODE(renderable));
        wr_transform_attach_child(mTransform, WR_NODE(transform));
        mRotationHandlesTransforms[i] = transform;
      }
    } else
      mHasRotationHandles = false;
  }
  if (mHasRotationHandles) {
    mActiveRotationHandleMaterial = wr_phong_material_new();
    const float color[3] = {0.0f, 0.0f, 0.0f};
    wr_phong_material_set_color(mActiveRotationHandleMaterial, color);
    wr_material_set_default_program(mActiveRotationHandleMaterial, WbWrenShaders::simpleShader());

    // Rotation line
    const float tailVertices[6] = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    WrStaticMesh *rotationLineMesh = wr_static_mesh_line_set_new(2, tailVertices, NULL);
    mMeshes.push_back(rotationLineMesh);

    WrRenderable *rotationLineRenderable = wr_renderable_new();
    mRenderables.push_back(rotationLineRenderable);
    wr_renderable_set_cast_shadows(rotationLineRenderable, false);
    wr_renderable_set_receive_shadows(rotationLineRenderable, false);
    wr_renderable_set_drawing_mode(rotationLineRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
    wr_renderable_set_visibility_flags(rotationLineRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
    wr_renderable_set_mesh(rotationLineRenderable, WR_MESH(rotationLineMesh));
    wr_renderable_set_material(rotationLineRenderable, mActiveRotationHandleMaterial, NULL);
    wr_renderable_set_drawing_order(rotationLineRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

    mRotationLineTransform = wr_transform_new();
    wr_transform_attach_child(mRotationLineTransform, WR_NODE(rotationLineRenderable));

    // Double arrow (left arrow, line, right arrow)
    const float doubleArrowVertices[30] = {-2.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f,  -1.0f,  -1.0f, 0.0f,  -1.0f,
                                           0.25f, 0.0f, 1.0f, 0.25f, 0.0f, 1.0f,  -0.25f, 0.0f,  -1.0f, -0.25f,
                                           0.0f,  2.0f, 0.0f, 0.0f,  1.0f, -1.0f, 0.0f,   1.0f,  1.0f,  0.0f};
    const float doubleArrowNormals[30] = {0};
    const unsigned int doubleArrowIndices[24] = {0, 1, 2, 2, 1, 0, 3, 4, 5, 5, 4, 3, 3, 5, 6, 6, 5, 3, 7, 8, 9, 9, 8, 7};
    WrStaticMesh *doubleArrowMesh = wr_static_mesh_new(30, 24, doubleArrowVertices, doubleArrowNormals, doubleArrowNormals,
                                                       doubleArrowNormals, doubleArrowIndices, false);
    mMeshes.push_back(doubleArrowMesh);

    WrRenderable *doubleArrowRenderable = wr_renderable_new();
    mRenderables.push_back(doubleArrowRenderable);
    wr_renderable_set_cast_shadows(doubleArrowRenderable, false);
    wr_renderable_set_receive_shadows(doubleArrowRenderable, false);
    wr_renderable_set_visibility_flags(doubleArrowRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
    wr_renderable_set_mesh(doubleArrowRenderable, WR_MESH(doubleArrowMesh));
    wr_renderable_set_material(doubleArrowRenderable, mActiveRotationHandleMaterial, NULL);
    wr_renderable_set_drawing_order(doubleArrowRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

    mRotationDoubleArrowTransform = wr_transform_new();
    wr_transform_attach_child(mRotationDoubleArrowTransform, WR_NODE(doubleArrowRenderable));

    WrTransform *root = wr_scene_get_root(wr_scene_get_instance());
    wr_transform_attach_child(root, WR_NODE(mRotationLineTransform));
    wr_transform_attach_child(root, WR_NODE(mRotationDoubleArrowTransform));

    wr_node_set_visible(WR_NODE(mRotationLineTransform), false);
    wr_node_set_visible(WR_NODE(mRotationDoubleArrowTransform), false);
  }
}

WbTranslateRotateManipulator::~WbTranslateRotateManipulator() {
  for (WrStaticMesh *mesh : mMeshes)
    wr_static_mesh_delete(mesh);

  for (WrRenderable *renderable : mRenderables) {
    // Delete picking material
    wr_material_delete(wr_renderable_get_material(renderable, "picking"));
    wr_node_delete(WR_NODE(renderable));
  }

  wr_node_delete(WR_NODE(mAxesTransform));

  if (mHasRotationHandles) {
    wr_material_delete(mActiveRotationHandleMaterial);
    wr_node_delete(WR_NODE(mRotationLineTransform));
    wr_node_delete(WR_NODE(mRotationDoubleArrowTransform));
  }

  for (int i = 0; i < 3; ++i) {
    if (mHasTranslationHandles)
      wr_node_delete(WR_NODE(mTranslationHandlesTransforms[i]));

    if (mHasRotationHandles)
      wr_node_delete(WR_NODE(mRotationHandlesTransforms[i]));

    if (mHasRotationHandles || mHasTranslationHandles) {
      wr_material_delete(mHandlesMaterials[i][0]);
      wr_material_delete(mHandlesMaterials[i][1]);
      wr_node_delete(WR_NODE(mInfiniteAxesTransforms[i]));
    }
  }
}

void WbTranslateRotateManipulator::highlightAxis(int index) {
  WbWrenAbstractManipulator::highlightAxis(index);

  for (int i = 0; i < 3; ++i) {
    if (mHasRotationHandles)
      wr_node_set_visible(WR_NODE(mRotationHandlesTransforms[i]), false);

    if (mHasTranslationHandles)
      wr_node_set_visible(WR_NODE(mTranslationHandlesTransforms[i]), false);
  }

  int handleIndex = index % 3;
  if (index < 3 && mHasTranslationHandles)
    wr_node_set_visible(WR_NODE(mTranslationHandlesTransforms[handleIndex]), true);
  else if (mHasRotationHandles)
    wr_node_set_visible(WR_NODE(mRotationHandlesTransforms[handleIndex]), true);

  if (mHasTranslationHandles || mHasRotationHandles) {
    wr_phong_material_set_transparency(mHandlesMaterials[handleIndex][1], 0.0f);
    wr_node_set_visible(WR_NODE(mInfiniteAxesTransforms[handleIndex]), true);
  }
}

void WbTranslateRotateManipulator::showNormal() {
  WbWrenAbstractManipulator::showNormal();

  if (mHasRotationHandles || mHasTranslationHandles)
    wr_node_set_visible(WR_NODE(mAxesTransform), true);

  for (int i = 0; i < 3; ++i) {
    if (mHasRotationHandles)
      wr_node_set_visible(WR_NODE(mRotationHandlesTransforms[i]), true);

    if (mHasTranslationHandles)
      wr_node_set_visible(WR_NODE(mTranslationHandlesTransforms[i]), true);

    if (mHasRotationHandles || mHasTranslationHandles) {
      wr_node_set_visible(WR_NODE(mInfiniteAxesTransforms[i]), false);
      wr_phong_material_set_transparency(mHandlesMaterials[i][1], HANDLES_TRANSPARENCY);
    }
  }
}

void WbTranslateRotateManipulator::showRotationLine(bool show) {
  assert(mRotationLineTransform && mRotationDoubleArrowTransform);
  if (!mHasRotationHandles)
    return;
  wr_node_set_visible(WR_NODE(mRotationLineTransform), show);
  wr_node_set_visible(WR_NODE(mRotationDoubleArrowTransform), show);
}

void WbTranslateRotateManipulator::updateRotationLine(const WbVector3 &begin, const WbVector3 &end,
                                                      const WbRotation &orientation, float arrowScale) {
  // Scale the double arrow size
  WrViewport *viewport = wr_scene_get_viewport(wr_scene_get_instance());
  const float width = wr_viewport_get_width(viewport), height = wr_viewport_get_height(viewport),
              maxDimension = height > width ? height : width, arrowScaleFactor = (arrowScale * 2.0f / maxDimension) * 10.0f;
  const float new_scale[3] = {arrowScaleFactor, arrowScaleFactor, arrowScaleFactor};
  wr_transform_set_scale(mRotationDoubleArrowTransform, new_scale);

  float tail[6];
  const double *data = begin.ptr();
  for (int i = 0; i < 6; ++i) {
    if (i > 2)
      data = end.ptr();

    tail[i] = static_cast<float>(data[i % 3]);
  }

  const float scaleFactor = (end - begin).length();
  const float scale[3] = {scaleFactor, scaleFactor, scaleFactor};

  // Line
  wr_transform_set_position(mRotationLineTransform, tail);
  wr_transform_set_scale(mRotationLineTransform, scale);

  // Double arrow
  wr_transform_set_position(mRotationDoubleArrowTransform, tail + 3);

  // Orientation
  WbVector3 baseX, baseY, baseZ;
  baseY = (end - begin).normalized();

  // Check if the vector is parallel to the xy-plane of the camera (dot product close to zero)
  if (fabs(baseY.dot(orientation.direction())) < 1e-6)
    baseX = baseY.cross(orientation.direction()).normalized();
  else {
    baseZ = orientation.up().cross(baseY).normalized();
    baseX = baseY.cross(baseZ).normalized();
  }
  baseZ = baseX.cross(baseY).normalized();

  WbRotation rotation(baseX, baseY, baseZ);
  rotation.normalize();

  float rotationArray[4];
  rotation.toFloatArray(rotationArray);
  wr_transform_set_orientation(mRotationLineTransform, rotationArray);
  wr_transform_set_orientation(mRotationDoubleArrowTransform, rotationArray);
}

WbVector3 WbTranslateRotateManipulator::relativeHandlePosition(int handleNumber) const {
  int axis = handleNumber % 3;
  WbVector3 position = STANDARD_COORDINATE_VECTORS[axis];
  if (handleNumber > 2)
    position[axis] -= 0.1f;

  return position * mScale;
}
