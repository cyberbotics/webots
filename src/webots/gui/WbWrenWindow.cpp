// Copyright 1996-2018 Cyberbotics Ltd.
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

#include "WbWrenWindow.hpp"

#include "WbDragSolidEvent.hpp"
#include "WbLensFlare.hpp"
#include "WbLightRepresentation.hpp"
#include "WbMessageBox.hpp"
#include "WbMultimediaStreamer.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbSysInfo.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenBloom.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenOpenGlContext.hpp"
#include "WbWrenPicker.hpp"
#include "WbWrenPostProcessingEffects.hpp"
#include "WbWrenShaders.hpp"
#include "WbWrenTextureOverlay.hpp"

#include <wren/config.h>
#include <wren/frame_buffer.h>
#include <wren/gl_state.h>
#include <wren/scene.h>
#include <wren/texture_rtt.h>
#include <wren/viewport.h>

#include <QtWidgets/QApplication>

#include <cassert>

WbWrenWindow *WbWrenWindow::cInstance = NULL;

const int PBO_COUNT = 2;

WbWrenWindow *WbWrenWindow::instance() {
  assert(cInstance);
  return cInstance;
}

WbWrenWindow::WbWrenWindow() :
  QWindow(),
  mUpdatePending(false),
  mSnapshotBuffer(NULL),
  mSnapshotBufferWidth(0),
  mSnapshotBufferHeight(0),
  mVideoPBOIndex(-1),
  mWrenMainFrameBuffer(NULL),
  mWrenMainFrameBufferTexture(NULL),
  mWrenNormalFrameBufferTexture(NULL),
  mWrenDepthFrameBufferTexture(NULL) {
  assert(WbWrenWindow::cInstance == NULL);
  WbWrenWindow::cInstance = this;

  setSurfaceType(QWindow::OpenGLSurface);

  QSurfaceFormat format = requestedFormat();
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
  format.setRedBufferSize(8);
  format.setGreenBufferSize(8);
  format.setBlueBufferSize(8);
#ifndef __APPLE__  // specifying alpha buffer size on macOS causes flickering when resizing the viewport
  format.setAlphaBufferSize(8);
#endif
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSwapInterval(0);
  setFormat(format);

  WbWrenOpenGlContext::init(this, this, format);
  if (!WbWrenOpenGlContext::instance()->isValid())
    WbMessageBox::critical(
      "Webots could not initialize the rendering system.\n"
      "We strongly recommend you to install the latest graphics drivers.\n"
      "Please do also check that your graphics hardware meets the requirements specified in the User Guide.");
  assert(WbWrenOpenGlContext::instance()->isValid());
}

WbWrenWindow::~WbWrenWindow() {
  WbWrenOpenGlContext::makeWrenCurrent();
  WbWrenPostProcessingEffects::clearResources();

  if (mWrenMainFrameBuffer)
    wr_frame_buffer_delete(mWrenMainFrameBuffer);

  if (mWrenMainFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenMainFrameBufferTexture));

  if (mWrenNormalFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenNormalFrameBufferTexture));

  if (mWrenDepthFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenDepthFrameBufferTexture));

  mWrenMainFrameBuffer = NULL;
  mWrenMainFrameBufferTexture = NULL;
  mWrenNormalFrameBufferTexture = NULL;
  mWrenDepthFrameBufferTexture = NULL;

  wr_scene_destroy();

  // delete shaders on exit
  WbWrenShaders::deleteShaders();

#ifndef _WIN32
  destroy();
#endif

  WbWrenOpenGlContext::doneWren();
  WbWrenOpenGlContext::destroy();

  delete[] mSnapshotBuffer;
}

// A custom initialization function is used here,
// because using the real one implies some issues on Mac since Qt 5.2.1
// Calling winId() creates the actual window and passes it 2 times into QGLWidget
// Using a custom initialization allows to make sure OpenGL is initialized after the
// QWidget creation (so the winId() is correct)
void WbWrenWindow::initialize() {
  if (wr_gl_state_is_initialized())
    return;

  // Moving these calls into the constructor causes rendering issues on Windows
  create();

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_scene_init(wr_scene_get_instance());

  // Useful for debugging
  // wr_config_set_bounding_volume_program(WbWrenShaders::boundingVolumeShader());
  // wr_config_set_show_axis_aligned_bounding_boxes(true);
  // wr_config_set_show_shadow_axis_aligned_bounding_boxes(true);
  // wr_config_set_show_bounding_spheres(true);

  // Workaround an OpenGL driver bug occuring in VMWare virtual machines:
  // - The OpenGL state when calling the glDrawElements function may be corrupted.
  //   In such case, the previous vertex buffers may be overriden with the current material.
  wr_config_set_requires_flush_after_draw(WbSysInfo::isVirtualMachine());

  // Workaround an OpenGL driver bug occuring in VMWare virtual machines:
  // - The OpenGL depth buffer returns the square root of the expected value when getting the depth buffer.
  wr_config_set_requires_depth_buffer_distortion(WbSysInfo::isVirtualMachine());

  updateFrameBuffer();

  wr_scene_set_fog_program(wr_scene_get_instance(), WbWrenShaders::fogShader());
  wr_scene_set_shadow_volume_program(wr_scene_get_instance(), WbWrenShaders::shadowVolumeShader());

  WbSysInfo::setOpenGLRenderer(QString(wr_gl_state_get_renderer()));

  WbWrenOpenGlContext::doneWren();
  WbWrenPostProcessingEffects::loadResources();
  updateWrenViewportDimensions();
}

void WbWrenWindow::updateWrenViewportDimensions() {
  // Fix rendering on display having a pixel ratio != 1.0
  // A retina display can be simulated:
  // http://stackoverflow.com/questions/12124576/how-to-simulate-a-retina-display-hidpi-mode-in-mac-os-x-10-8-mountain-lion-on

  const float ratio = (float)devicePixelRatio();  // qreal => float in prevision to be sent to wren.
  WbWrenPicker::setScreenRatio(ratio);
  WbWrenTextureOverlay::setScreenRatio(ratio);
  WbDragPhysicsEvent::setScreenRatio(ratio);
  WbWrenBloom::setScreenRatio(ratio);
  WbViewpoint *viewpoint = WbWorld::instance() ? WbWorld::instance()->viewpoint() : NULL;
  if (viewpoint)
    viewpoint->updatePostProcessingEffects();
}

void WbWrenWindow::blitMainFrameBufferToScreen() {
  wr_frame_buffer_blit_to_screen(mWrenMainFrameBuffer);
}

void WbWrenWindow::renderLater() {
  if (!mUpdatePending) {
    mUpdatePending = true;
    QApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}

void WbWrenWindow::renderNow() {
  if (!isExposed() || !wr_gl_state_is_initialized())
    return;

#ifdef __APPLE__
  // Make sure all events are processed before first render, omitting this snippet
  // causes graphical corruption on macOS due to the main framebuffer being invalid.
  // On Windows, this fix causes a crash on startup for certain worlds.
  static int first = true;
  if (first) {
    first = false;
    QCoreApplication::processEvents(QEventLoop::AllEvents);
  }
#endif

  WbPerformanceLog *log = WbPerformanceLog::instance();
  if (log)
    log->startMeasure(WbPerformanceLog::MAIN_RENDERING);

  WbViewpoint *viewpoint = WbWorld::instance() ? WbWorld::instance()->viewpoint() : NULL;
  if (viewpoint) {
    viewpoint->enableNodeVisibility(false);
    viewpoint->updatePostProcessingParameters();
  }

  WbWrenOpenGlContext::makeWrenCurrent();

  wr_scene_render(wr_scene_get_instance(), NULL);

  WbWrenOpenGlContext::instance()->swapBuffers(this);
  WbWrenOpenGlContext::doneWren();

  WbMultimediaStreamer *multimediaStreamer = WbMultimediaStreamer::instance();
  if (multimediaStreamer->isReady())
    feedMultimediaStreamer();

  if (log)
    log->stopMeasure(WbPerformanceLog::MAIN_RENDERING);
}

bool WbWrenWindow::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::UpdateRequest:
      if (mUpdatePending) {
        mUpdatePending = false;
        renderNow();
      }
      return true;
    case QEvent::Expose:
    case QEvent::Move:
    case QEvent::Resize:
      resizeWren(width(), height());
      return true;
    default:
      return QWindow::event(event);
  }
}

void WbWrenWindow::resizeWren(int width, int height) {
  if (!wr_gl_state_is_initialized())
    return;

  WbWrenOpenGlContext::makeWrenCurrent();

  int w = width;
  int h = height;

  const qreal ratio = devicePixelRatio();
  if (ratio != 1.0) {
    w *= ratio;
    h *= ratio;
  }

  wr_viewport_set_size(wr_scene_get_viewport(wr_scene_get_instance()), w, h);

  updateFrameBuffer();

  WbWrenTextureOverlay::updateOverlayDimensions();
  WbWrenLabelOverlay::updateOverlaysDimensions();
  WbLightRepresentation::updateScreenScale(width, height);

  if (WbWorld::instance() && WbWorld::instance()->viewpoint())
    WbWorld::instance()->viewpoint()->updatePostProcessingEffects();

  WbWrenOpenGlContext::doneWren();

  renderLater();

  emit resized();
}

void WbWrenWindow::flipImageBuffer(unsigned char *buffer, int width, int height, int channels) {
  // flip vertically the image (about 2x faster than QImage::mirrored())
  const int lineSize = width * channels;
  const int halfHeight = height / 2;
  unsigned char *srcPtr = buffer;
  unsigned char *dstPtr = &buffer[(height - 1) * lineSize];
  for (int y = 0; y < halfHeight; y++) {
    for (int x = 0; x < lineSize; x++) {
      const uchar d = dstPtr[x];
      dstPtr[x] = srcPtr[x];
      srcPtr[x] = d;
    }
    srcPtr += lineSize;
    dstPtr -= lineSize;
  }
}

QImage WbWrenWindow::grabWindowBufferNow() {
  WbWrenOpenGlContext::makeWrenCurrent();

  const qreal ratio = devicePixelRatio();
  int w = width() * ratio;
  int h = height() * ratio;
  if (mSnapshotBuffer == NULL || w != mSnapshotBufferWidth || h != mSnapshotBufferHeight) {
    mSnapshotBufferWidth = h;
    mSnapshotBufferHeight = w;
    delete[] mSnapshotBuffer;
    mSnapshotBuffer = new unsigned char[4 * w * h];
  }
  readPixels(w, h, GL_BGRA, mSnapshotBuffer);
  flipImageBuffer(mSnapshotBuffer, w, h, 4);

  WbWrenOpenGlContext::doneWren();

  return QImage(mSnapshotBuffer, w, h, QImage::Format_RGB32);
}

void WbWrenWindow::initVideoPBO() {
  WbWrenOpenGlContext::makeWrenCurrent();

  const qreal ratio = devicePixelRatio();
  mVideoWidth = width() * ratio;
  mVideoHeight = height() * ratio;
  const int size = 4 * mVideoWidth * mVideoHeight;
  wr_scene_init_frame_capture(wr_scene_get_instance(), PBO_COUNT, mVideoPBOIds, size);
  mVideoPBOIndex = -1;

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::completeVideoPBOProcessing(bool canceled) {
  WbWrenOpenGlContext::makeWrenCurrent();

  // process last frame
  if (canceled)
    mPBOMutexes[mVideoPBOIndex].unlock();
  else
    processVideoPBO();
  mVideoPBOIndex = -1;
  wr_scene_terminate_frame_capture(wr_scene_get_instance());

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::processVideoPBO() {
  if (mVideoPBOIndex < 0)
    return;

  WbWrenOpenGlContext::makeWrenCurrent();

  // Process previously copied pixels
  WrScene *scene = wr_scene_get_instance();
  wr_scene_bind_pixel_buffer(scene, mVideoPBOIds[mVideoPBOIndex]);
  unsigned char *buffer = (unsigned char *)wr_scene_map_pixel_buffer(scene, GL_READ_ONLY);
  mPBOMutexes[mVideoPBOIndex].unlock();
  if (buffer) {
    emit videoImageReady(buffer, mVideoPBOIndex);
    wr_scene_unmap_pixel_buffer(scene);
  }

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::updateFrameBuffer() {
  WbWrenOpenGlContext::makeWrenCurrent();

  if (mWrenMainFrameBuffer)
    wr_frame_buffer_delete(mWrenMainFrameBuffer);

  if (mWrenMainFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenMainFrameBufferTexture));

  if (mWrenNormalFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenNormalFrameBufferTexture));

  if (mWrenDepthFrameBufferTexture)
    wr_texture_delete(WR_TEXTURE(mWrenDepthFrameBufferTexture));

  int w = width();
  int h = height();

  const qreal ratio = devicePixelRatio();
  if (ratio != 1.0) {
    w *= ratio;
    h *= ratio;
  }

  mWrenMainFrameBuffer = wr_frame_buffer_new();
  wr_frame_buffer_set_size(mWrenMainFrameBuffer, w, h);

  mWrenMainFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenMainFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB16F);

  mWrenNormalFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenNormalFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_RGB8);

  wr_frame_buffer_append_output_texture(mWrenMainFrameBuffer, mWrenMainFrameBufferTexture);
  wr_frame_buffer_append_output_texture(mWrenMainFrameBuffer, mWrenNormalFrameBufferTexture);
  wr_frame_buffer_enable_depth_buffer(mWrenMainFrameBuffer, true);

  mWrenDepthFrameBufferTexture = wr_texture_rtt_new();
  wr_texture_set_internal_format(WR_TEXTURE(mWrenDepthFrameBufferTexture), WR_TEXTURE_INTERNAL_FORMAT_DEPTH24_STENCIL8);
  wr_frame_buffer_set_depth_texture(mWrenMainFrameBuffer, mWrenDepthFrameBufferTexture);

  wr_frame_buffer_setup(mWrenMainFrameBuffer);
  wr_viewport_set_frame_buffer(wr_scene_get_viewport(wr_scene_get_instance()), mWrenMainFrameBuffer);

  WbWrenOpenGlContext::doneWren();
}

void WbWrenWindow::requestGrabWindowBuffer() {
  WbWrenOpenGlContext::makeWrenCurrent();

  // Asynchronous pixels copy from GPU
  if (mVideoPBOIndex >= 0)
    // process previous frame image stored in PBO
    processVideoPBO();

  WrScene *scene = wr_scene_get_instance();

  mVideoPBOIndex = (mVideoPBOIndex + 1) % PBO_COUNT;
  // Request pixels copy
  // read pixels from framebuffer to PBO: wr_scene_get_main_buffer() should return immediately
  mPBOMutexes[mVideoPBOIndex].lock();
  wr_scene_bind_pixel_buffer(scene, mVideoPBOIds[mVideoPBOIndex]);
  readPixels(mVideoWidth, mVideoHeight, GL_BGRA, 0);
  wr_scene_bind_pixel_buffer(scene, 0);

  WbWrenOpenGlContext::doneWren();
}

QSize WbWrenWindow::minimumSize() const {
  return QSize(1, 1);
}

QSize WbWrenWindow::sizeHint() const {
  return QSize(400, 400);
}

void WbWrenWindow::feedMultimediaStreamer() {
  // Skip the first call to 'renderNow()' because OpenGL
  // context seems to be not ready. Not skipping causes
  // a freeze.
  static bool skipFirstFrame = true;
  if (skipFirstFrame)
    skipFirstFrame = false;
  else {
    WbMultimediaStreamer *multimediaStreamer = WbMultimediaStreamer::instance();
    void *buffer = multimediaStreamer->buffer();
    readPixels(multimediaStreamer->imageWidth(), multimediaStreamer->imageHeight(), GL_RGB, buffer);
    multimediaStreamer->sendImage();
  }
}

void WbWrenWindow::readPixels(int width, int height, unsigned int format, void *buffer) {
#ifdef __linux__
  if (WbSysInfo::isVirtualMachine()) {
    // Reading the front buffer is not supported by all OpenGL implementations (especially on Linux running in a VM).
    // In that case, to read the front buffer, we need to swap the buffers, read the back buffer and swap the buffers again.
    // However, doing this may cause flickering on platforms where reading the front buffer is supported (including macOS).
    WbWrenOpenGlContext::instance()->swapBuffers(this);
    wr_scene_get_main_buffer(wr_scene_get_instance(), width, height, format, GL_UNSIGNED_BYTE, GL_BACK, buffer);
    WbWrenOpenGlContext::instance()->swapBuffers(this);
  } else
#endif
    wr_scene_get_main_buffer(wr_scene_get_instance(), width, height, format, GL_UNSIGNED_BYTE, GL_FRONT, buffer);
}
