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

#include "WbLightRepresentation.hpp"

#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/shader_program.h>
#include <wren/static_mesh.h>
#include <wren/texture.h>
#include <wren/texture_2d.h>
#include <wren/transform.h>
#include <wren/viewport.h>

#include <QtCore/QFileInfo>
#include <QtGui/QImage>

static const int ICON_SIZE = 32;  // Size of light icon in pixels

void WbLightRepresentation::updateScreenScale(int width, int height) {
  const float screenScale = 2.0f * ICON_SIZE / (width > height ? width : height);
  WrShaderProgram *shader = WbWrenShaders::lightRepresentationShader();
  wr_shader_program_set_custom_uniform_value(shader, "screenScale", WR_SHADER_PROGRAM_UNIFORM_TYPE_FLOAT,
                                             reinterpret_cast<const char *>(&screenScale));
}

WbLightRepresentation::WbLightRepresentation(WrTransform *parent, const WbVector3 &position) :
  mPosition(position),
  mQImage(NULL) {
  // Load sun texture (512x512 ARGB)
  QByteArray imagePath = QFileInfo("gl:textures/light_representation.png").absoluteFilePath().toUtf8();
  mTexture = wr_texture_2d_copy_from_cache(imagePath.constData());
  if (!mTexture) {
    mQImage = new QImage(imagePath.constData());
    assert(!mQImage->isNull());

    mTexture = wr_texture_2d_new();
    wr_texture_set_translucent(WR_TEXTURE(mTexture), true);
    wr_texture_set_size(WR_TEXTURE(mTexture), mQImage->width(), mQImage->height());
    wr_texture_2d_set_data(mTexture, reinterpret_cast<const char *>(mQImage->bits()));
    wr_texture_2d_set_file_path(mTexture, imagePath);
    wr_texture_setup(WR_TEXTURE(mTexture));
  }

  mMaterial = wr_phong_material_new();
  wr_material_set_texture(mMaterial, WR_TEXTURE(mTexture), 0);
  wr_material_set_default_program(mMaterial, WbWrenShaders::lightRepresentationShader());

  const float vertices[12] = {-0.5f, -0.5f, 0.0f, -0.5f, 0.5f, 0.0f, 0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f};
  // Null normals as we wont use them
  const float normals[12] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const float textureCoordinates[8] = {
    0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
  };
  const unsigned int indices[6] = {2, 1, 0, 0, 3, 2};
  mMesh = wr_static_mesh_new(4, 6, vertices, normals, textureCoordinates, textureCoordinates, indices, false);

  mRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderable, false);
  wr_renderable_set_receive_shadows(mRenderable, false);
  wr_renderable_set_material(mRenderable, mMaterial, NULL);
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_LIGHTS_POSITIONS);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
  wr_renderable_set_drawing_order(mRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mTransform), false);

  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  wr_transform_attach_child(parent, WR_NODE(mTransform));

  WrViewport *viewport = wr_scene_get_viewport(wr_scene_get_instance());
  updateScreenScale(wr_viewport_get_width(viewport), wr_viewport_get_height(viewport));
}

WbLightRepresentation::~WbLightRepresentation() {
  wr_node_delete(WR_NODE(mTransform));
  wr_node_delete(WR_NODE(mRenderable));
  wr_material_delete(mMaterial);
  wr_static_mesh_delete(mMesh);
  wr_texture_delete(WR_TEXTURE(mTexture));
  delete mQImage;
}

void WbLightRepresentation::setPosition(const WbVector3 &position) {
  float positionArray[3];
  position.toFloatArray(positionArray);
  wr_transform_set_position(mTransform, positionArray);
  mPosition = position;
}

void WbLightRepresentation::setVisible(bool visible) {
  wr_node_set_visible(WR_NODE(mTransform), visible);
}
