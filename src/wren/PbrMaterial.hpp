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

#ifndef PBR_MATERIAL_HPP
#define PBR_MATERIAL_HPP

#include "Cache.hpp"
#include "Material.hpp"

#include <unordered_map>

namespace wren {

  // Class to represent a PBR material
  class PbrMaterial : public Material {
  public:
    WrMaterialType type() const override { return WR_MATERIAL_PBR; }

    void setTexture(Texture *texture, size_t index) override;
    void setTextureCubeMap(TextureCubeMap *texture, size_t index) override;

    // Encapsulate memory management
    static PbrMaterial *createMaterial();
    static void deleteMaterial(PbrMaterial *material);
    static size_t cachedItemCount() { return PbrMaterial::cCache.size(); }
    static void printCacheContents();

    void setTransparency(float transparency) override;
    void setBackgroundColor(const glm::vec3 &backgroundColor);
    void setBaseColor(const glm::vec3 &baseColor);
    void setEmissiveColor(const glm::vec3 &emissiveColor);
    void setRoughness(float roughness);
    void setMetalness(float metalness);
    void setIblStrength(float iblStrength);
    void setNormalMapFactor(float normalMapFactor);
    void setOcclusionMapStrength(float occlusionMapStrength);
    void setEmissiveIntensity(float emissiveIntensity);
    void setAllParameters(const glm::vec3 &backgroundColor, const glm::vec3 &baseColor, float transparency, float roughness,
                          float metalness, float iblStrength, float normalMapFactor, float occlusionMapStrength,
                          const glm::vec3 &emissiveColor, float emissiveIntensity);

    // Clear all the data but preserve the program
    void clearMaterial() override;
    void bind(bool bindProgram = true) const override;
    size_t sortingId() const override;
    void updateTranslucency() override;

  private:
    PbrMaterial();
    ~PbrMaterial();
    void init();

    static std::unordered_map<cache::Key, cache::PbrMaterialData> cCache;
    void releaseMaterial() override;
    void updateMaterial(const GlslLayout::PbrMaterial &material);

    cache::Key mCacheKey;
    cache::PbrMaterialData *mCacheData;
  };

}  // namespace wren

#endif  // PBR_MATERIAL_HPP
