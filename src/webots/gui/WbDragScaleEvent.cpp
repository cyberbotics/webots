// Copyright 1996-2021 Cyberbotics Ltd.
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

#include "WbDragScaleEvent.hpp"

#include "WbAbstractTransform.hpp"
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
    new WbResizeCommand(mCylinder, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
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
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the capsule

WbRescaleCapsuleEvent::WbRescaleCapsuleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                             WbGeometry *selectedGeometry) :
  WbResizeCapsuleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleCapsuleEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mCapsule, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
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
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the box

WbRescaleBoxEvent::WbRescaleBoxEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                     WbGeometry *selectedGeometry) :
  WbResizeBoxEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleBoxEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mBox, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
}

void WbRescaleBoxEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector3 &rescaledSize = mResizeRatio * mBox->size();
  mBox->setSize(rescaledSize);
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the plane

WbRescalePlaneEvent::WbRescalePlaneEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                         WbGeometry *selectedGeometry) :
  WbResizePlaneEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescalePlaneEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mPlane, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
}

void WbRescalePlaneEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector2 &rescaledSize = mResizeRatio * mPlane->size();

  if (exceedsFloatMax(rescaledSize.x()) || exceedsFloatMax(rescaledSize.y())) {
    emit aborted();
    return;
  }

  mPlane->setSize(rescaledSize);
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the cone

WbRescaleConeEvent::WbRescaleConeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                       WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCone(static_cast<WbCone *>(selectedGeometry)) {
}

void WbRescaleConeEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mCone, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
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
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the elevation grid

WbRescaleElevationGridEvent::WbRescaleElevationGridEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                         int handleNumber, WbGeometry *selectedGeometry) :
  WbResizeElevationGridEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleElevationGridEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mElevationGrid, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
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

  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Drags scaling the indexed face set

WbRescaleIndexedFaceSetEvent::WbRescaleIndexedFaceSetEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                           int handleNumber, WbGeometry *selectedGeometry) :
  WbResizeIndexedFaceSetEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

void WbRescaleIndexedFaceSetEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbResizeCommand(mIndexedFaceSet, WbVector3(mAbsoluteScaleRatio, mAbsoluteScaleRatio, mAbsoluteScaleRatio)));
}

void WbRescaleIndexedFaceSetEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  if (exceedsFloatMax(mResizeRatio * mIndexedFaceSet->range(mCoordinate))) {
    emit aborted();
    return;
  }

  mIndexedFaceSet->rescale(WbVector3(mResizeRatio, mResizeRatio, mResizeRatio));

  // update global resize values
  mAbsoluteScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mAbsoluteScaleRatio, mViewDistanceUnscaling);
}

// Moves a scale handle by dragging the mouse and changes the scale field accordingly //
////////////////////////////////////////////////////////////////////////////////////////

WbDragScaleHandleEvent::WbDragScaleHandleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                               WbAbstractTransform *selectedTransform) :
  WbDragView3DEvent(viewpoint),
  mTransform(selectedTransform),
  mHandleNumber(handleNumber),
  mManipulator(selectedTransform->scaleManipulator()),
  mScaleRatio(1.0f),
  mTotalScale(1.0f) {
  mCoordinate = handleNumber;
  mManipulator->highlightAxis(mManipulator->coordinate(mHandleNumber));
  mManipulator->setActive(true);
  mViewDistanceUnscaling = mViewpoint->viewDistanceUnscaling(mTransform->position());

  // Compute mouse position offset
  WbVector3 localMousePosition;
  computeHandlesPositions(initialMousePosition, mAttachedHandlePosition, mOppositeHandlePosition, localMousePosition);
  mViewpoint->toPixels(mAttachedHandlePosition, mAttachedHandleProjection);
  mMousePositionOffset = WbVector2(initialMousePosition.x(), initialMousePosition.y()) - mAttachedHandleProjection;
  mLocalMouseOffset = localMousePosition[mCoordinate] -
                      (mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate]);

  mInitialScale = WbVariant(mTransform->scale());
  mViewpoint->lock();
}

WbDragScaleHandleEvent::~WbDragScaleHandleEvent() {
  mTransform->updateTranslateRotateHandlesSize();
  mTransform->setResizeManipulatorDimensions();
  mManipulator->setActive(false);
  mManipulator->showNormal();
  mManipulator->updateHandleDimensions(1.0f, 1.0f);

  mViewpoint->unlock();
}

void WbDragScaleHandleEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(
    new WbEditCommand(mTransform->scaleFieldValue(), mInitialScale, mTransform->scaleFieldValue()->variantValue()));
}

void WbDragScaleHandleEvent::computeHandlesPositions(const QPoint &currentMousePosition, WbVector3 &attachedHandlePos,
                                                     WbVector3 &oppositeHandlePos, WbVector3 &localMousePos) {
  const WbMatrix4 &matrix = mTransform->matrix();

  WbMatrix3 unscaledMatrix = matrix.extracted3x3Matrix();
  WbVector3 absoluteScale = matrix.scale();
  unscaledMatrix.scale(1.0f / absoluteScale.x(), 1.0f / absoluteScale.y(), 1.0f / absoluteScale.z());

  attachedHandlePos = matrix.translation() + unscaledMatrix * (mTotalScale * mViewDistanceUnscaling *
                                                               mManipulator->relativeHandlePosition(mHandleNumber));
  oppositeHandlePos = 2.0 * matrix.translation() - attachedHandlePos;

  const float zEye = mViewpoint->zEye(attachedHandlePos);
  localMousePos = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), zEye);
  localMousePos = matrix.pseudoInversed(localMousePos);
  localMousePos /= absoluteScale;
}

void WbDragScaleHandleEvent::computeRatio(const QPoint &currentMousePosition) {
  WbVector3 localMousePosition;
  computeHandlesPositions(currentMousePosition, mAttachedHandlePosition, mOppositeHandlePosition, localMousePosition);

  WbVector2 currentPosition(currentMousePosition.x(), currentMousePosition.y());
  mViewpoint->toPixels(mAttachedHandlePosition, mAttachedHandleProjection, mOppositeHandlePosition, mOppositeHandleProjection);
  currentPosition -= mAttachedHandleProjection + mMousePositionOffset;
  WbVector2 difference(mAttachedHandleProjection - mOppositeHandleProjection);
  const double length = difference.length();
  mScaleRatio = 1.0;

  if (length > 0.0) {
    difference.normalize();
    mScaleRatio = 1.0 + currentPosition.dot(difference) / length;
  }

  if (mScaleRatio <= 0.01)
    mScaleRatio = 1.0;
}

void WbDragScaleHandleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector3 &previousScale = mTransform->scale();
  mTotalScale *= mScaleRatio;
  mTransform->setScale(mCoordinate, mScaleRatio * previousScale[mCoordinate]);
  mManipulator->updateHandleDimensions(mTotalScale, mViewDistanceUnscaling);
}

// uniform scale

WbUniformScaleEvent::WbUniformScaleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                         WbAbstractTransform *selectedTransform) :
  WbDragScaleHandleEvent(initialMousePosition, viewpoint, handleNumber, selectedTransform) {
}

void WbUniformScaleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector3 &s = mScaleRatio * mTransform->scale();
  mTotalScale *= mScaleRatio;
  mTransform->setScale(s.rounded(WbPrecision::GUI_MEDIUM));
  mManipulator->updateHandleDimensions(mTotalScale, mViewDistanceUnscaling);
}
