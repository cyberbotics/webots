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

#include "TextureTransform.hpp"

#include "Debug.hpp"

#include <wren/texture_transform.h>

#include <cmath>

namespace wren {

  TextureTransform::TextureTransform() :
    mIsMatrixDirty(false),
    mMatrix(gMat4Identity),
    mScale(glm::vec3(1.0)),
    mPosition(gVec3Zeros),
    mCenter(gVec3Zeros),
    mRotation(0.0f) {
  }

  TextureTransform::~TextureTransform() {
    for (auto &material : mMaterialsUsingThisInstance)
      material->setTextureTransform(NULL);
  }

  void TextureTransform::setCenter(float center_x, float center_y) {
    mCenter = glm::vec3(center_x, center_y, 0.0f);
    mIsMatrixDirty = true;
  }

  void TextureTransform::setPosition(float position_x, float position_y) {
    mPosition = glm::vec3(position_x, position_y, 0.0f);
    mIsMatrixDirty = true;
  }

  void TextureTransform::setRotation(float rotation) {
    mRotation = rotation;
    mIsMatrixDirty = true;
  }

  void TextureTransform::setScale(float scale_x, float scale_y) {
    mScale = glm::vec3(scale_x, scale_y, 1.0f);
    mIsMatrixDirty = true;
  }

  const glm::mat4 &TextureTransform::matrix() const {
    updateMatrix();
    return mMatrix;
  }

  void TextureTransform::updateMatrix() const {
    if (!mIsMatrixDirty)
      return;

    // GLM matrices are column-major
    glm::mat4 tM(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, mPosition.x, -mPosition.y, 0.0f, 1.0f);

    // y coordinate of OpenGL is inverted and has an offset of one
    glm::mat4 cM(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, mCenter.x, -mCenter.y - 1.0f, 0.0f,
                 1.0f);

    glm::mat4 minusCM(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -mCenter.x, mCenter.y + 1.0f,
                      0.0f, 1.0f);

    glm::mat4 sM(mScale.x, 0.0f, 0.0f, 0.0f, 0.0f, mScale.y, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

    glm::mat4 rM(cosf(mRotation), -sinf(mRotation), 0.0f, 0.0f, sinf(mRotation), cosf(mRotation), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);

    // Apply transform following X3D specs:
    // http://www.web3d.org/documents/specifications/19775-1/V3.2/Part01/components/texturing.html#TextureTransform
    mMatrix = minusCM * sM * rM * cM * tM;

    mIsMatrixDirty = false;
  }

  void TextureTransform::addMaterialUser(Material *material) {
    assert(material);
    for (const Material *user : mMaterialsUsingThisInstance) {
      if (user == material)
        return;
    }

    mMaterialsUsingThisInstance.push_back(material);
  }

  void TextureTransform::removeMaterialUser(Material *material) {
    assert(material);
    for (size_t i = 0; i < mMaterialsUsingThisInstance.size(); ++i) {
      if (mMaterialsUsingThisInstance[i] == material)
        mMaterialsUsingThisInstance.erase(mMaterialsUsingThisInstance.begin() + i);
    }
  }

}  // namespace wren

// C interface implementation
WrTextureTransform *wr_texture_transform_new() {
  return reinterpret_cast<WrTextureTransform *>(wren::TextureTransform::createTextureTransform());
}

void wr_texture_transform_delete(WrTextureTransform *transform) {
  wren::TextureTransform::deleteTextureTransform(reinterpret_cast<wren::TextureTransform *>(transform));
}

void wr_texture_transform_set_center(WrTextureTransform *transform, float center_x, float center_y) {
  reinterpret_cast<wren::TextureTransform *>(transform)->setCenter(center_x, center_y);
}

void wr_texture_transform_set_position(WrTextureTransform *transform, float position_x, float position_y) {
  reinterpret_cast<wren::TextureTransform *>(transform)->setPosition(position_x, position_y);
}

void wr_texture_transform_set_rotation(WrTextureTransform *transform, float rotation) {
  reinterpret_cast<wren::TextureTransform *>(transform)->setRotation(rotation);
}

void wr_texture_transform_set_scale(WrTextureTransform *transform, float scale_x, float scale_y) {
  reinterpret_cast<wren::TextureTransform *>(transform)->setScale(scale_x, scale_y);
}

void wr_texture_transform_apply_to_uv_coordinate(WrTextureTransform *transform, float *coord) {
  const glm::vec2 result = reinterpret_cast<wren::TextureTransform *>(transform)->applyToUvCoordinate(glm::make_vec2(coord));
  coord[0] = result.x;
  coord[1] = result.y;
}
