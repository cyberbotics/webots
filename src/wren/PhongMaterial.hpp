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

#ifndef PHONG_MATERIAL_HPP
#define PHONG_MATERIAL_HPP

#include "Cache.hpp"
#include "Material.hpp"

#include <unordered_map>

namespace wren {

  // Class to represent a Blinn-Phong material
  class PhongMaterial : public Material {
  public:
    WrMaterialType type() const override { return WR_MATERIAL_PHONG; }

    void setTexture(Texture *texture, size_t index) override;

    // Encapsulate memory management
    static PhongMaterial *createMaterial();
    static void deleteMaterial(PhongMaterial *material);
    static size_t cachedItemCount() { return PhongMaterial::cCache.size(); }
    static void printCacheContents();

    // Clear all the data but preserve the program
    void clearMaterial() override;

    void setColor(const glm::vec3 &color) override;
    void setAmbient(const glm::vec3 &ambient, bool linearColor);
    void setDiffuse(const glm::vec3 &diffuse, bool linearColor);
    void setSpecular(const glm::vec3 &specular);
    void setEmissive(const glm::vec3 &emissive);
    void setShininess(float shininess);
    void setSpecularExponent(float specularExponent);
    void setTransparency(float transparency) override;
    void setAllParameters(const glm::vec3 &ambient, const glm::vec3 &diffuse, const glm::vec3 &specular,
                          const glm::vec3 &emissive, float shininess, float transparency);

    void setColorPerVertex(bool enabled) override { mColorPerVertex = enabled; }

    void bind(bool bindProgram = true) const override;
    size_t sortingId() const override;
    void updateTranslucency() override;

  private:
    PhongMaterial();
    ~PhongMaterial();
    void init();

    static std::unordered_map<cache::Key, cache::PhongMaterialData> cCache;
    void updateUniforms() const override;
    void releaseMaterial() override;
    void updateMaterial(const GlslLayout::PhongMaterial &material);

    bool mColorPerVertex;
    cache::Key mCacheKey;
    cache::PhongMaterialData *mCacheData;
  };

}  // namespace wren

#endif  // PHONG_MATERIAL_HPP
