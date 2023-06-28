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

#include "WbWrenPicker.hpp"

#include "WbWrenOpenGlContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/material.h>
#include <wren/scene.h>

// Setup & attach picking material, based on the unique ID
// ID is encoded in the following way:
// Most significant word: red and green channels of ambient color
// Least significant word: red and green channels of diffuse color
// These are combined in RGBA channels in the picking fragment shader
void WbWrenPicker::setPickable(WrRenderable *renderable, int uniqueId, bool pickable) {
  WrMaterial *material = wr_renderable_get_material(renderable, "picking");

  if (!material) {
    material = wr_phong_material_new();
    wr_material_set_default_program(material, WbWrenShaders::pickingShader());

    wr_renderable_set_material(renderable, material, "picking");
  }

  float encodedId[6] = {0.0f};

  if (pickable) {
    // ID is incremented since a 0 value would mean that no object was picked
    // (ID = 0 is valid)
    int id = uniqueId + 1;

    for (int i = 4; i >= 0; --i) {
      if (i == 2)
        continue;

      encodedId[i] = (id & 0x000000FF) / 255.0f;
      id >>= 8;
    }
  }

  wr_phong_material_set_linear_ambient(material, encodedId);
  wr_phong_material_set_linear_diffuse(material, encodedId + 3);
}

WbWrenPicker::WbWrenPicker() : mSelectedId(-1), mPickedTranslation(0), mPickedRotation(0), mPickedScale(0), mPickedResize(0) {
  mViewport = wr_viewport_new();

  const float color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  wr_viewport_set_clear_color_rgba(mViewport, color);

  setup();
}

WbWrenPicker::~WbWrenPicker() {
  cleanup();
  wr_viewport_delete(mViewport);
}

void WbWrenPicker::setup() {
  WbWrenOpenGlContext::makeWrenCurrent();

  WrViewport *viewport = wr_scene_get_viewport(wr_scene_get_instance());
  mWidth = wr_viewport_get_width(viewport);
  mHeight = wr_viewport_get_height(viewport);
  wr_viewport_set_size(mViewport, mWidth, mHeight);

  mFrameBuffer = wr_frame_buffer_new();
  mOutputTexture = wr_texture_rtt_new();

  wr_frame_buffer_set_size(mFrameBuffer, mWidth, mHeight);
  wr_frame_buffer_enable_depth_buffer(mFrameBuffer, true);
  wr_frame_buffer_append_output_texture(mFrameBuffer, mOutputTexture);
  wr_frame_buffer_enable_copying(mFrameBuffer, 0, true);
  wr_frame_buffer_setup(mFrameBuffer);

  wr_viewport_set_frame_buffer(mViewport, mFrameBuffer);
  wr_viewport_set_camera(mViewport, wr_viewport_get_camera(viewport));

  WbWrenOpenGlContext::doneWren();
}

void WbWrenPicker::cleanup() {
  WbWrenOpenGlContext::makeWrenCurrent();

  wr_texture_delete(WR_TEXTURE(mOutputTexture));
  wr_frame_buffer_delete(mFrameBuffer);

  WbWrenOpenGlContext::doneWren();
}

bool WbWrenPicker::hasSizeChanged() {
  WrViewport *viewport = wr_scene_get_viewport(wr_scene_get_instance());
  const int width = wr_viewport_get_width(viewport);
  const int height = wr_viewport_get_height(viewport);

  if (mWidth != width || mHeight != height) {
    mWidth = width;
    mHeight = height;
    return true;
  }
  return false;
}

bool WbWrenPicker::pick(int x, int y) {
  WbWrenOpenGlContext::makeWrenCurrent();
  mCoordinates.setXyz(0.0, 0.0, 0.0);
  mSelectedId = -1;
  mPickedTranslation = 0;
  mPickedRotation = 0;
  mPickedScale = 0;
  mPickedResize = 0;

  // Recreate framebuffer and textures if viewport's size has changed
  if (hasSizeChanged()) {
    cleanup();
    setup();
  }

  // Check if object was picked & decode ID
  WrScene *scene = wr_scene_get_instance();
  wr_viewport_enable_skybox(mViewport, false);
  wr_scene_enable_translucence(scene, false);
  wr_scene_enable_depth_reset(scene, false);
  wr_scene_render_to_viewports(scene, 1, &mViewport, "picking", true);
  wr_scene_enable_depth_reset(scene, true);
  wr_viewport_enable_skybox(mViewport, true);
  wr_scene_enable_translucence(scene, true);

  char *data = new char[4];
  wr_frame_buffer_copy_pixel(mFrameBuffer, 0, x, y, data, true);

  unsigned char *data2 = reinterpret_cast<unsigned char *>(data);
  int id = (data2[0] << 24) | (data2[1] << 16) | (data2[2] << 8) | data2[3];

  delete[] data;

  if (id == 0) {
    WbWrenOpenGlContext::doneWren();
    return false;
  } else
    mSelectedId = id - 1;

  if ((mSelectedId & 0x7FFFFFF0) == 0x7FFFFFF0) {
    int axis = mSelectedId & 0x00000003, type = mSelectedId & HANDLES_RESIZE;
    mSelectedId = -1;
    switch (type) {
      case HANDLES_TRANSLATE:
        mPickedTranslation = axis;
        break;
      case HANDLES_ROTATE:
        mPickedRotation = axis;
        break;
      case HANDLES_SCALE:
        mPickedScale = axis;
        break;
      case HANDLES_RESIZE:
        mPickedResize = axis;
        break;
      default:
        assert(false);
    }
    WbWrenOpenGlContext::doneWren();
    return true;
  }

  // Compute coordinates
  float depth;
  wr_frame_buffer_copy_depth_pixel(mFrameBuffer, x, y, &depth, true);

  WbWrenOpenGlContext::doneWren();

  mCoordinates = WbVector3(x, mHeight - y - 1, depth);

  return true;
}
