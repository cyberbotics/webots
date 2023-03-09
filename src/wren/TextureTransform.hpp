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

#ifndef TEXTURE_TRANSFORM_HPP
#define TEXTURE_TRANSFORM_HPP

#include "Constants.hpp"

#include <vector>

namespace wren {

  class Material;
  // Transform matrix that can be applied on texture coordinates.
  class TextureTransform {
  public:
    static TextureTransform *createTextureTransform() { return new TextureTransform(); }
    static void deleteTextureTransform(TextureTransform *transform) { delete transform; }

    void setCenter(float center_x, float center_y);
    void setPosition(float position_x, float position_y);
    void setRotation(float rotation);
    void setScale(float scale_x, float scale_y);
    const glm::vec2 center() const { return glm::vec2(mCenter); }
    const glm::vec2 position() const { return glm::vec2(mPosition); }
    float rotation() const { return mRotation; }
    const glm::vec2 scale() const { return glm::vec2(mScale); }
    const glm::mat4 &matrix() const;  // will recompute the matrix if dirty
    glm::vec2 applyToUvCoordinate(const glm::vec2 &coord) const { return glm::vec2(matrix() * glm::vec4(coord, 0.0, 1.0)); }
    const bool isDirty() const { return mIsMatrixDirty; }

    void addMaterialUser(Material *material);
    void removeMaterialUser(Material *material);

  private:
    TextureTransform();
    ~TextureTransform();

    void updateMatrix() const;

    // May be modified when getting matrix, thus mutable
    mutable bool mIsMatrixDirty;
    mutable glm::mat4 mMatrix;

    glm::vec3 mScale;
    glm::vec3 mPosition;
    glm::vec3 mCenter;
    float mRotation;
    std::vector<Material *> mMaterialsUsingThisInstance;
  };

}  // namespace wren

#endif  // TEXTURE_TRANSFORM_HPP
