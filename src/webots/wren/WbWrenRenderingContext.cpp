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

#include "WbWrenRenderingContext.hpp"

#include <wren/config.h>
#include <wren/gl_state.h>

WbWrenRenderingContext *WbWrenRenderingContext::cRenderingContext = NULL;

void WbWrenRenderingContext::setWrenRenderingContext(int width, int height) {
  cleanup();
  cRenderingContext = new WbWrenRenderingContext(width, height);
}

void WbWrenRenderingContext::cleanup() {
  delete cRenderingContext;
}

const double WbWrenRenderingContext::SOLID_LINE_SCALE_FACTOR = 0.1;

WbWrenRenderingContext::WbWrenRenderingContext(int width, int height) :
  QObject(),
  mWidth(width),
  mHeight(height),
  mLineScale(0.0),
  mSolidLineScale(0.0),
  mRenderingMode(RM_PLAIN),
  mProjectionMode(PM_PERSPECTIVE),
  mOptionalRenderingsMask(VM_MAIN) {
  assert(VM_WEBOTS_RANGE_CAMERA == VM_REGULAR + (unsigned int)VF_LASER_BEAM);
}

WbWrenRenderingContext::~WbWrenRenderingContext() {
  if (cRenderingContext)
    cRenderingContext = NULL;
}

void WbWrenRenderingContext::setDimension(int width, int height) {
  mWidth = width;
  mHeight = height;
  emit dimensionChanged();
}

void WbWrenRenderingContext::setLineScale(float lineScale) {
  wr_config_set_line_scale(lineScale);
  mSolidLineScale = SOLID_LINE_SCALE_FACTOR * lineScale;

  emit lineScaleChanged();
}

bool WbWrenRenderingContext::isOptionalRenderingEnabled(int optionalRendering) const {
  return (optionalRendering & mOptionalRenderingsMask) != 0;
}

unsigned int WbWrenRenderingContext::visibilityMask() const {
  int visibilityMask = mOptionalRenderingsMask;
  if (mRenderingMode == RM_WIREFRAME && isOptionalRenderingEnabled(VF_ALL_BOUNDING_OBJECTS))
    visibilityMask &= ~VM_REGULAR;
  return visibilityMask;
}

void WbWrenRenderingContext::enableOptionalRendering(int optionalRendering, bool enable, bool userAction) {
  if (enable)
    mOptionalRenderingsMask |= optionalRendering;
  else
    mOptionalRenderingsMask &= ~optionalRendering;

  emit optionalRenderingChanged(optionalRendering);
  if (userAction)
    emit view3dRefreshRequired();
}

void WbWrenRenderingContext::setRenderingMode(int renderingMode, bool userAction) {
  mRenderingMode = renderingMode;
  emit renderingModeChanged();
  if (userAction)
    emit view3dRefreshRequired();
}

void WbWrenRenderingContext::setProjectionMode(int projectionMode, bool userAction) {
  mProjectionMode = projectionMode;
  emit projectionModeChanged();
  if (userAction)
    emit view3dRefreshRequired();
}

void WbWrenRenderingContext::requestView3dRefresh() {
  emit view3dRefreshRequired();
}

bool WbWrenRenderingContext::isIntelRenderer() const {
  if (!wr_gl_state_is_initialized())
    return false;

  const char *vendor = wr_gl_state_get_vendor();
  return (strncasecmp(vendor, "Intel", 5) == 0);
}

bool WbWrenRenderingContext::isAmdRenderer() const {
  if (!wr_gl_state_is_initialized())
    return false;

  const char *vendor = wr_gl_state_get_vendor();
  return (strncasecmp(vendor, "Amd", 3) == 0 || strncasecmp(vendor, "ATI", 3) == 0);
}

bool WbWrenRenderingContext::isNvidiaRenderer() const {
  if (!wr_gl_state_is_initialized())
    return false;

  const char *vendor = wr_gl_state_get_vendor();
  return (strncasecmp(vendor, "Nvidia", 6) == 0);
}

bool WbWrenRenderingContext::isMesaRenderer() const {
  if (!wr_gl_state_is_initialized())
    return false;

  const char *vendor = wr_gl_state_get_vendor();
  return (strncasecmp(vendor, "Mesa", 4) == 0);
}

bool WbWrenRenderingContext::isMicrosoftRenderer() const {
  if (!wr_gl_state_is_initialized())
    return false;

  const char *vendor = wr_gl_state_get_vendor();
  return (strncasecmp(vendor, "Microsoft", 9) == 0);
}
