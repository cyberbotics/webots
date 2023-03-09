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

#ifndef VIEWPORT_HPP
#define VIEWPORT_HPP

#include "Camera.hpp"
#include "Constants.hpp"

#include <wren/viewport.h>

namespace wren {

  class FrameBuffer;
  class Overlay;
  class TextureRtt;
  class PostProcessingEffect;

  // A Viewport is defined by its width and height, the Camera it uses for rendering and
  // the FrameBuffer it renders into. If no FrameBuffer is set, it renders to the main window.
  class Viewport {
  public:
    // Encapsulate memory management
    static Viewport *createViewport() { return new Viewport(); }
    static void deleteViewport(Viewport *viewport) { delete viewport; }

    void setClearColor(const glm::vec4 &clearColor) { mClearColor = clearColor; }
    void setPolygonMode(WrViewportPolygonMode polygonMode) { mPolygonMode = polygonMode; }
    void setVisbilityMask(int mask) { mVisibilityMask = mask; }
    void setSize(int width, int height);
    void setPixelRatio(int ratio) { mPixelRatio = ratio; }
    void setCamera(Camera *camera);
    void setFrameBuffer(FrameBuffer *frameBuffer);
    void enableShadows(bool enable) { mAreShadowsEnabled = enable; }
    void enableSkybox(bool enable) { mIsSkyboxEnabled = enable; }
    void enableAspectRatioSync(bool enable) { mSyncAspectRatio = enable; }

    void attachOverlay(Overlay *overlay);
    void detachOverlay(Overlay *overlay);
    void renderOverlay(Overlay *overlay);

    void addPostProcessingEffect(PostProcessingEffect *postProcessingEffect);
    void removePostProcessingEffect(PostProcessingEffect *postProcessingEffect);
    void setAmbientOcclusionEffect(PostProcessingEffect *postProcessingEffect) {
      mAmbientOcclusionEffect = postProcessingEffect;
    }
    void setAntiAliasingEffect(PostProcessingEffect *postProcessingEffect) { mAntiAliasingEffect = postProcessingEffect; }

    int width() const { return mWidth; }
    int height() const { return mHeight; }
    int pixelRatio() const { return mPixelRatio; }
    bool areShadowsEnabled() const { return mAreShadowsEnabled; }
    WrViewportPolygonMode polygonMode() const { return mPolygonMode; }
    int visibilityMask() const { return mVisibilityMask; }
    Camera *camera() const { return mCamera; }
    FrameBuffer *frameBuffer() const { return mFrameBuffer; }
    bool isSkyboxEnabled() const { return mIsSkyboxEnabled; }

    void updateUniforms() const;
    void bind() const;
    void clear() const;
    void finishRender() const;
    void applyPostProcessing();
    void applyAmbientOcclusion();
    void applyAntiAliasing();
    void drawOverlays();

  private:
    Viewport();
    ~Viewport();

    void sortOverlays();

    mutable bool mIsSizeDirty;
    bool mSyncAspectRatio;

    glm::vec4 mClearColor;
    WrViewportPolygonMode mPolygonMode;
    int mVisibilityMask;
    int mWidth;
    int mHeight;
    int mPixelRatio;

    Camera *mCamera;
    FrameBuffer *mFrameBuffer;

    bool mAreShadowsEnabled;
    bool mIsSkyboxEnabled;

    std::vector<Overlay *> mOverlays;
    std::vector<PostProcessingEffect *> mPostProcessingEffects;
    PostProcessingEffect *mAmbientOcclusionEffect;
    PostProcessingEffect *mAntiAliasingEffect;
  };

}  // namespace wren

#endif  // VIEWPORT_HPP
