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

#include "WbWrenWindow.hpp"

#include "WbDragSolidEvent.hpp"
#include "WbLensFlare.hpp"
#include "WbLightRepresentation.hpp"
#include "WbLog.hpp"
#include "WbMessageBox.hpp"
#include "WbMultimediaStreamingServer.hpp"
#include "WbPerformanceLog.hpp"
#include "WbPreferences.hpp"
#include "WbSysInfo.hpp"
#include "WbVideoRecorder.hpp"
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
  mWrenDepthFrameBufferTexture(NULL),
  mVideoStreamingServer(NULL) {
  assert(WbWrenWindow::cInstance == NULL);
  WbWrenWindow::cInstance = this;

  setSurfaceType(QWindow::OpenGLSurface);

  const WbVersion openGLTargetVersion(3, 3);

  QSurfaceFormat format = requestedFormat();
  format.setVersion(openGLTargetVersion.majorNumber(), openGLTargetVersion.minorNumber());
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
    WbLog::fatal(tr("Webots could not initialize the rendering system.\n"
                    "Please check your GPU abilities and install the latest graphics drivers.\n"
                    "Please do also check that your graphics hardware meets the requirements specified in the User Guide."));

  const WbVersion openGLActualVersion(WbWrenOpenGlContext::instance()->format().majorVersion(),
                                      WbWrenOpenGlContext::instance()->format().minorVersion());

  if (openGLActualVersion < openGLTargetVersion)
    WbLog::fatal(tr("Webots requires OpenGL %1 while only OpenGL %2 can be initialized.\n"
                    "Please check your GPU abilities and install the latest graphics drivers.\n"
                    "Please do also check that your graphics hardware meets the requirements specified in the User Guide.")
                   .arg(openGLTargetVersion.toString(false))
                   .arg(openGLActualVersion.toString(false)));
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

  WbWrenOpenGlContext::doneWren();
  WbWrenPostProcessingEffects::loadResources();
  updateWrenViewportDimensions();
}

void WbWrenWindow::updateWrenViewportDimensions() {
  const int ratio = (int)devicePixelRatio();
  wr_viewport_set_pixel_ratio(wr_scene_get_viewport(wr_scene_get_instance()), ratio);
  WbVideoRecorder::instance()->setScreenPixelRatio(ratio);
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

void WbWrenWindow::renderNow(bool culling) {
  if (!isExposed() || !wr_gl_state_is_initialized())
    return;

  static int first = true;
#ifdef __APPLE__
  // Make sure all events are processed before first render, omitting this snippet
  // causes graphical corruption on macOS due to the main framebuffer being invalid.
  // On Windows, this fix causes a crash on startup for certain worlds.
  if (first)
    QCoreApplication::processEvents(QEventLoop::AllEvents);
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

  wr_scene_render(wr_scene_get_instance(), NULL, culling);

  WbWrenOpenGlContext::instance()->swapBuffers(this);
  WbWrenOpenGlContext::doneWren();

  if (mVideoStreamingServer && mVideoStreamingServer->isNewFrameNeeded() && !first)
    // Skip the first call to 'renderNow()' because OpenGL context seems to be not ready. Not skipping causes a freeze.
    mVideoStreamingServer->sendImage(grabWindowBufferNow());

  if (log)
    log->stopMeasure(WbPerformanceLog::MAIN_RENDERING);

  if (first)
    first = false;
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

void WbWrenWindow::flipAndScaleDownImageBuffer(const unsigned char *source, unsigned char *destination, int sourceWidth,
                                               int sourceHeight, int scaleDownFactor) {
  // flip vertically the image and scale it down (about 3x faster than QImage::mirrored(), QImage::scaled())
  // cppcheck-suppress unsafeClassDivZero
  const int h = sourceHeight / scaleDownFactor;
  const int w = sourceWidth / scaleDownFactor;
  const int yFactor = scaleDownFactor * sourceWidth;

  // - The `unsigned char *` to `int *` cast is possible assuming that a pixel is coded as four bytes (RGBA)
  //   aligned on an `int *` boundary.
  // - A preliminary `unsigned char *` to `void *` cast is required to by-pass "cast-align" clang warnings.
  const uint32_t *src = static_cast<const uint32_t *>(static_cast<void *>(const_cast<unsigned char *>(source)));
  uint32_t *dst = static_cast<uint32_t *>(static_cast<void *>(destination));

  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++)
      dst[(h - 1 - y) * w + x] = src[y * yFactor + x * scaleDownFactor];
  }
}

QImage WbWrenWindow::grabWindowBufferNow() {
  WbWrenOpenGlContext::makeWrenCurrent();

  const int destinationWidth = width();
  const int destinationHeight = height();
  if (mSnapshotBuffer == NULL || destinationWidth != mSnapshotBufferWidth || destinationHeight != mSnapshotBufferHeight) {
    delete[] mSnapshotBuffer;
    mSnapshotBufferWidth = destinationWidth;
    mSnapshotBufferHeight = destinationHeight;
    mSnapshotBuffer = new unsigned char[4 * destinationWidth * destinationHeight];
  }
  const qreal ratio = devicePixelRatio();
  const int sourceWidth = destinationWidth * ratio;
  const int sourceHeight = destinationHeight * ratio;
  unsigned char *temp = new unsigned char[4 * sourceWidth * sourceHeight];
  readPixels(sourceWidth, sourceHeight, GL_BGRA, temp);
  flipAndScaleDownImageBuffer(temp, mSnapshotBuffer, sourceWidth, sourceHeight, ratio);
  delete[] temp;
  WbWrenOpenGlContext::doneWren();

  return QImage(mSnapshotBuffer, mSnapshotBufferWidth, mSnapshotBufferHeight, QImage::Format_RGB32);
}

void WbWrenWindow::initVideoPBO() {
  WbWrenOpenGlContext::makeWrenCurrent();

  const int ratio = (int)devicePixelRatio();
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
  if (!canceled)
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
  unsigned char *buffer = static_cast<unsigned char *>(wr_scene_map_pixel_buffer(scene, GL_READ_ONLY));
  if (buffer) {
    emit videoImageReady(buffer);
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

  mWrenMainFrameBuffer = wr_frame_buffer_new();
  wr_frame_buffer_set_size(mWrenMainFrameBuffer, width(), height());

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

  wr_viewport_set_size(wr_scene_get_viewport(wr_scene_get_instance()), width(), height());

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

void WbWrenWindow::setVideoStreamingServer(WbMultimediaStreamingServer *streamingServer) {
  mVideoStreamingServer = streamingServer;
  connect(mVideoStreamingServer, &WbMultimediaStreamingServer::imageRequested, this, &WbWrenWindow::feedMultimediaStreamer);
}

void WbWrenWindow::feedMultimediaStreamer() {
  renderNow();
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
