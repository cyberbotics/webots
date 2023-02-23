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

#ifndef RENDERABLE_HPP
#define RENDERABLE_HPP

#include "Constants.hpp"
#include "GlslLayout.hpp"
#include "Node.hpp"
#include "Primitive.hpp"

#include <string>
#include <unordered_map>

#include <wren/renderable.h>

namespace wren {

  class LightNode;
  class Material;
  class Mesh;
  class ShaderProgram;
  class ShadowVolumeCaster;

  // Container class consisting of a Material, a Mesh and a drawing mode.
  // Inherits from Node and can be attached to a Transform for positioning.
  // During rendering, the Mesh's vertex coordinates are multiplied by the Transform's matrix.
  class Renderable : public Node {
  public:
    // Encapsulate memory management
    static Renderable *createRenderable() { return new Renderable(); }

    static void setUseMaterial(const char *materialName) { Renderable::cUseMaterialName = materialName; }
    static const char *useMaterial() { return Renderable::cUseMaterialName; }

    void setDefaultMaterial(Material *material) { mDefaultMaterial = material; }
    void setEffectiveMaterial(Material *material) { mEffectiveMaterial = material; }
    void setOptionalMaterial(std::string name, Material *material) { mOptionalMaterials[name] = material; }

    // To ensure a valid bounding sphere, only set a Mesh to a Renderable after having called Mesh::setup.
    void setMesh(Mesh *mesh);
    void setDrawingMode(WrRenderableDrawingMode drawingMode) { mDrawingMode = drawingMode; }
    void setDrawingOrder(WrRenderableDrawingOrder drawingOrder) { mDrawingOrder = drawingOrder; }
    void setVisibilityFlags(int flags) { mVisibilityFlags = flags; }
    void setCastShadows(bool castShadows);
    void setReceiveShadows(bool receiveShadows) { mReceiveShadows = receiveShadows; }
    void setSceneCulling(bool culling) { mSceneCulling = culling; }
    void setInViewSpace(bool inViewSpace) { mInViewSpace = inViewSpace; }
    void setZSortedRendering(bool zSortedRendering) { mZSortedRendering = zSortedRendering; }
    void setFaceCulling(bool faceCulling) { mFaceCulling = faceCulling; }
    void setPointSize(float pointSize) { mPointSize = pointSize; }
    void setInvertFrontFace(bool invertFrontFace) { mInvertFrontFace = invertFrontFace; }

    const glm::mat4 &parentMatrix() const;
    Material *defaultMaterial() const { return mDefaultMaterial; }
    Material *effectiveMaterial() const { return mEffectiveMaterial; }
    Material *optionalMaterial(const std::string &name) const;
    Mesh *mesh() const { return mMesh; }
    int visibilityFlags() const { return mVisibilityFlags; }
    bool castShadows() const { return mCastShadows; }
    bool receiveShadows() const { return mReceiveShadows; }
    bool sceneCulling() const { return mSceneCulling; }
    ShadowVolumeCaster *shadowVolumeCaster() { return mShadowVolumeCaster; }
    WrRenderableDrawingMode drawingMode() const { return mDrawingMode; }
    WrRenderableDrawingOrder drawingOrder() const { return mDrawingOrder; }
    bool isInViewSpace() const { return mInViewSpace; }
    bool zSortedRendering() const;
    bool invertFrontFace() const { return mInvertFrontFace; }
    void render(const ShaderProgram *program = NULL);
    void renderWithoutMaterial(const ShaderProgram *program);

    void recomputeBoundingSphereInViewSpace(const glm::mat4 &viewMatrix);
    const primitive::Sphere &boundingSphereInViewSpace() const { return mBoundingSphereInViewSpace; }

    bool isTranslucent() const;
    size_t sortingId() const;

    // Updates model matrix using parent transform
    void updateFromParent() override;
    const primitive::Aabb &aabb() override;
    const primitive::Sphere &boundingSphere() const override;
    void setMatrixDirty() const override;

  private:
    static const char *cUseMaterialName;

    Renderable();
    virtual ~Renderable();

    void setupAndRender(const ShaderProgram *program);
    void updateShadowVolumeCaster();
    void computeMostInfluentialLights();

    void recomputeAabb() const override;
    void recomputeBoundingSphere() const override;

    Material *mDefaultMaterial;
    Material *mEffectiveMaterial;
    std::unordered_map<std::string, Material *> mOptionalMaterials;

    Mesh *mMesh;

    ShadowVolumeCaster *mShadowVolumeCaster;

    WrRenderableDrawingMode mDrawingMode;
    WrRenderableDrawingOrder mDrawingOrder;
    int mVisibilityFlags;
    bool mCastShadows;
    bool mReceiveShadows;
    bool mSceneCulling;
    bool mInViewSpace;
    bool mZSortedRendering;
    bool mFaceCulling;
    bool mInvertFrontFace;
    float mPointSize;

    primitive::Sphere mBoundingSphereInViewSpace;
  };

}  // namespace wren

#endif  // RENDERABLE_HPP
