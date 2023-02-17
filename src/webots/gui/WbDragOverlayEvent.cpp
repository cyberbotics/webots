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

#include "WbDragOverlayEvent.hpp"

#include "WbRenderingDevice.hpp"
#include "WbWrenTextureOverlay.hpp"

WbDragOverlayEvent::WbDragOverlayEvent(const QPoint &initialMousePosition, WbRenderingDevice *renderingDevice) :
  mInitialMousePosition(initialMousePosition),
  mRenderingDevice(renderingDevice) {
}

WbDragTranslateOverlayEvent::WbDragTranslateOverlayEvent(const QPoint &initialMousePosition, const QPoint &windowSize,
                                                         WbRenderingDevice *renderingDevice) :
  WbDragOverlayEvent(initialMousePosition, renderingDevice),
  mWindowSize(windowSize) {
  mHalfWidth = (double)mRenderingDevice->width() * mRenderingDevice->pixelSize() * 0.5f / (double)mWindowSize.x();
  mHalfHeight = (double)mRenderingDevice->height() * mRenderingDevice->pixelSize() * 0.5f / (double)mWindowSize.y();
}

void WbDragTranslateOverlayEvent::apply(const QPoint &currentMousePosition) {
  QPoint difference = currentMousePosition - mInitialMousePosition;
  mInitialMousePosition = currentMousePosition;
  mRenderingDevice->moveWindow(difference.x(), difference.y());
}

WbDragResizeOverlayEvent::WbDragResizeOverlayEvent(const QPoint &initialMousePosition, WbRenderingDevice *renderingDevice) :
  WbDragOverlayEvent(initialMousePosition, renderingDevice) {
  mTextureWidthInv = 1.0 / renderingDevice->width();
  mTextureHeightInv = 1.0 / renderingDevice->height();
  int overlayWidth, overlayHeight;
  mRenderingDevice->overlay()->size(overlayWidth, overlayHeight);
  mInitialOverlaySize = QPoint(overlayWidth, overlayHeight);
  renderingDevice->overlay()->enableDefaultSizeBackground(true);
}

WbDragResizeOverlayEvent::~WbDragResizeOverlayEvent() {
  mRenderingDevice->overlay()->enableDefaultSizeBackground(false);
}

void WbDragResizeOverlayEvent::apply(const QPoint &currentMousePosition) {
  QPoint difference = mInitialOverlaySize + currentMousePosition - mInitialMousePosition;
  double scaleX = difference.x() * mTextureWidthInv;
  double scaleY = difference.y() * mTextureHeightInv;
  mRenderingDevice->setPixelSize(scaleX > scaleY ? scaleX : scaleY);
}
