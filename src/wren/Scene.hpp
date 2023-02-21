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

#ifndef SCENE_HPP
#define SCENE_HPP

#include "GlslLayout.hpp"
#include "Primitive.hpp"

#include <vector>

#include <wren/scene.h>

namespace wren {

  class Camera;
  class DirectionalLight;
  class LightNode;
  class PointLight;
  class Renderable;
  class SpotLight;
  class Transform;
  class Viewport;
  class ShaderProgram;
  class ShadowVolumeCaster;

  // Contains all elements making up a Scene.
  // The same Scene may be rendered through different viewports at different points in time.
  class Scene {
  public:
    // The corresponding enum in scene.h needs to be kept up to date
    enum FogType { FOG_TYPE_NONE, FOG_TYPE_EXPONENTIAL, FOG_TYPE_EXPONENTIAL2, FOG_TYPE_LINEAR };

    // Defines the basis for the fog distance computation
    // The The corresponding enum in scene.h needs to be kept up to date
    enum FogDepthType { FOG_DEPTH_TYPE_PLANE, FOG_DEPTH_TYPE_POINT };

    static Scene *instance();
    static void destroy() { delete cInstance; }

    static void init();
    void reset();
    static void applyPendingUpdates();

    static void getMainBuffer(int width, int height, unsigned int format, unsigned int data_type, unsigned int buffer_type,
                              void *buffer);
    void initFrameCapture(int pixelBufferCount, unsigned int *pixelBufferIds, int frameSize);
    static void bindPixelBuffer(int buffer);
    void *mapPixelBuffer(unsigned int accessMode);
    static void unMapPixelBuffer();
    void terminateFrameCapture();

    void addLight(LightNode *light);
    void removeLight(LightNode *light);

    void setFog(WrSceneFogType fogType = WR_SCENE_FOG_TYPE_LINEAR,
                WrSceneFogDepthType depthType = WR_SCENE_FOG_DEPTH_TYPE_PLANE, const glm::vec4 &color = gVec4Ones,
                float density = 1.0f, float start = 0.0f, float end = 1.0f);
    void setSkybox(Renderable *renderable);
    void setHdrClearQuad(Renderable *renderable);

    void setFogProgram(ShaderProgram *program) { mFogProgram = program; }
    void setShadowVolumeProgram(ShaderProgram *program) { mShadowVolumeProgram = program; }

    void enableDepthReset(bool enabled) { mClearDepth = enabled; }
    void enableSkybox(bool enabled) { mRenderSkybox = enabled; }
    void enableHdrClear(bool enabled) { mHdrClear = enabled; }
    void enableTranslucence(bool enabled) { mTranslucence = enabled; }

    Transform *root() { return mRoot; }
    Viewport *mainViewport() { return mMainViewport; }
    Viewport *currentViewport() { return mCurrentViewport; }

    const std::vector<DirectionalLight *> &directionalLights() const { return mDirectionalLightsActive; }
    const std::vector<PointLight *> &pointLights() const { return mPointLightsActive; }
    const std::vector<SpotLight *> &spotLights() const { return mSpotLightsActive; }

    void enqueueRenderable(Renderable *renderable);

    int computeNodeCount() const;
    static void printSceneTree();
    void render(bool culling);
    void renderToViewports(std::vector<Viewport *> viewports, bool culling);

    void addFrameListener(void (*listener)()) { mListeners.push_back(listener); }
    void removeFrameListener(void (*listener)());

  private:
    typedef std::vector<Renderable *>::iterator RenderQueueIterator;
    typedef std::vector<ShadowVolumeCaster *>::iterator ShadowVolumeIterator;
    typedef std::vector<Renderable *> RenderQueue;

    Scene();
    ~Scene();

    void prepareRender();
    void renderToViewport(bool culling);
    void updateFogUniformBuffer();

    RenderQueueIterator partitionByVisibility(RenderQueueIterator first, RenderQueueIterator last);
    RenderQueueIterator partitionByViewability(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByTranslucency(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByUseMaterial(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByStencilProgram(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByShadowReceiving(RenderQueueIterator first, RenderQueueIterator last);
    static RenderQueueIterator partitionByZOrder(RenderQueueIterator first, RenderQueueIterator last);

    ShadowVolumeIterator partitionShadowsByVisibility(ShadowVolumeIterator first, ShadowVolumeIterator last, LightNode *light);

    static void sortRenderQueueByState(RenderQueueIterator first, RenderQueueIterator last);
    void sortRenderQueueByDistance(RenderQueueIterator first, RenderQueueIterator last);

    static void renderDefault(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest = false);
    void renderStencilPerLight(LightNode *light, RenderQueueIterator first, RenderQueueIterator firstShadowReceiver,
                               RenderQueueIterator last);
    void renderStencilShadowVolumesDepthPass(ShadowVolumeCaster *shadowVolume, LightNode *light);
    void renderStencilShadowVolumesDepthFail(ShadowVolumeCaster *shadowVolume, LightNode *light);
    static void renderStencilAmbientEmissive(RenderQueueIterator first, RenderQueueIterator last);
    static void renderStencilDiffuseSpecular(RenderQueueIterator first, RenderQueueIterator last, LightNode *light,
                                             bool applyShadows = true);
    void renderStencilFog(RenderQueueIterator first, RenderQueueIterator last);
    static void renderStencilWithoutProgram(RenderQueueIterator first, RenderQueueIterator last);
    static void renderTranslucent(RenderQueueIterator first, RenderQueueIterator last, bool disableDepthTest = false);

    static Scene *cInstance;

    size_t mFrameCounter;

    Transform *mRoot;
    Viewport *mMainViewport;     // main window viewport
    Viewport *mCurrentViewport;  // viewport currently being rendered to

    std::vector<DirectionalLight *> mDirectionalLightsActive;
    std::vector<PointLight *> mPointLightsActive;
    std::vector<SpotLight *> mSpotLightsActive;

    std::vector<DirectionalLight *> mDirectionalLightsInactive;
    std::vector<PointLight *> mPointLightsInactive;
    std::vector<SpotLight *> mSpotLightsInactive;

    GlslLayout::LightRenderable mLightRenderable;

    std::vector<void (*)()> mListeners;
    std::vector<ShadowVolumeCaster *> mShadowVolumeQueue;
    std::vector<RenderQueue> mRenderQueues;
    Renderable *mSkybox;
    Renderable *mHdrClearQuad;
    ShaderProgram *mFogProgram;
    ShaderProgram *mShadowVolumeProgram;

    bool mRenderSkybox;
    bool mHdrClear;
    bool mTranslucence;
    bool mClearDepth;

    bool mIsFogDirty;
    GlslLayout::Fog mFog;

    int mPixelBufferCount;
    unsigned int *mPixelBufferIds;
  };

}  // namespace wren

#endif  // SCENE_HPP
