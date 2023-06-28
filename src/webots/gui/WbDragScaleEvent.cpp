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

#include "WbDragScaleEvent.hpp"

#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCone.hpp"
#include "WbCylinder.hpp"
#include "WbEditCommand.hpp"
#include "WbElevationGrid.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbPlane.hpp"
#include "WbResizeAndTranslateCommand.hpp"
#include "WbResizeCommand.hpp"
#include "WbResizeManipulator.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWrenRenderingContext.hpp"

// Drags scaling the cylinder

WbRescaleCylinderEvent::WbRescaleCylinderEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                               WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCylinder(static_cast<WbCylinder *>(selectedGeometry)) {
}

void WbRescaleCylinderEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mCylinder, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleCylinderEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedRadius = mCylinder->radius() * mResizeRatio;

  if (exceedsFloatMax(resizedRadius)) {
    emit aborted();
    return;
  }

  mCylinder->setRadius(resizedRadius);

  const double resizedHeight = mCylinder->height() * mResizeRatio;

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCylinder->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the capsule

WbRescaleCapsuleEvent::WbRescaleCapsuleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                             WbGeometry *selectedGeometry) :
  WbResizeCapsuleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleCapsuleEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(new WbResizeCommand(mCapsule, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleCapsuleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedRadius = mCapsule->radius() * mResizeRatio;

  if (exceedsFloatMax(resizedRadius)) {
    emit aborted();
    return;
  }

  mCapsule->setRadius(resizedRadius);

  const double resizedHeight = mCapsule->height() * mResizeRatio;

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCapsule->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the box

WbRescaleBoxEvent::WbRescaleBoxEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                     WbGeometry *selectedGeometry) :
  WbResizeBoxEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleBoxEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(new WbResizeCommand(mBox, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleBoxEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector3 &rescaledSize = mResizeRatio * mBox->size();
  mBox->setSize(rescaledSize);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the plane

WbRescalePlaneEvent::WbRescalePlaneEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                         WbGeometry *selectedGeometry) :
  WbResizePlaneEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescalePlaneEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(new WbResizeCommand(mPlane, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescalePlaneEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector2 &rescaledSize = mResizeRatio * mPlane->size();

  if (exceedsFloatMax(rescaledSize.x()) || exceedsFloatMax(rescaledSize.y())) {
    emit aborted();
    return;
  }

  mPlane->setSize(rescaledSize);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the cone

WbRescaleConeEvent::WbRescaleConeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                       WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCone(static_cast<WbCone *>(selectedGeometry)) {
}

void WbRescaleConeEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(new WbResizeCommand(mCone, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleConeEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double resizedBottomRadius = fabs(mCone->bottomRadius() * mResizeRatio);

  if (exceedsFloatMax(resizedBottomRadius)) {
    emit aborted();
    return;
  }

  mCone->setBottomRadius(resizedBottomRadius);
  const double resizedHeight = fabs(mCone->height() * mResizeRatio);

  if (exceedsFloatMax(resizedHeight)) {
    emit aborted();
    return;
  }

  mCone->setHeight(resizedHeight);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the elevation grid

WbRescaleElevationGridEvent::WbRescaleElevationGridEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                         int handleNumber, WbGeometry *selectedGeometry) :
  WbResizeElevationGridEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleElevationGridEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mElevationGrid, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleElevationGridEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  const double resizedXspacing = mElevationGrid->xSpacing() * mResizeRatio;
  if (exceedsFloatMax(resizedXspacing)) {
    emit aborted();
    return;
  }

  mElevationGrid->setXspacing(resizedXspacing);

  const double resizedYspacing = mElevationGrid->ySpacing() * mResizeRatio;
  if (exceedsFloatMax(resizedYspacing)) {
    emit aborted();
    return;
  }

  mElevationGrid->setYspacing(resizedYspacing);

  if (exceedsFloatMax(mResizeRatio * mElevationGrid->heightRange())) {
    emit aborted();
    return;
  }

  mElevationGrid->setHeightScaleFactor(mResizeRatio);

  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the indexed face set

WbRescaleIndexedFaceSetEvent::WbRescaleIndexedFaceSetEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                           int handleNumber, WbGeometry *selectedGeometry) :
  WbResizeIndexedFaceSetEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleIndexedFaceSetEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mIndexedFaceSet, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbRescaleIndexedFaceSetEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  if (exceedsFloatMax(mResizeRatio * mIndexedFaceSet->range(mCoordinate))) {
    emit aborted();
    return;
  }

  mIndexedFaceSet->rescale(WbVector3(mResizeRatio, mResizeRatio, mResizeRatio));

  // update global resize values
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}
