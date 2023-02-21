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

#include "WbDragResizeEvent.hpp"

#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCone.hpp"
#include "WbCylinder.hpp"
#include "WbElevationGrid.hpp"
#include "WbGeometry.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbMatrix4.hpp"
#include "WbPlane.hpp"
#include "WbResizeAndTranslateCommand.hpp"
#include "WbResizeCommand.hpp"
#include "WbResizeManipulator.hpp"
#include "WbSphere.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWrenRenderingContext.hpp"

// Moves a resize handle by dragging the mouse and changes the geometry size accordingly //
///////////////////////////////////////////////////////////////////////////////////////////

WbDragResizeHandleEvent::WbDragResizeHandleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                                 WbGeometry *selectedGeometry) :
  WbDragView3DEvent(viewpoint),
  mInitialMousePosition(initialMousePosition),
  mSelectedGeometry(selectedGeometry),
  mHandleNumber(handleNumber),
  mManipulator(selectedGeometry->resizeManipulator()),
  mResizeRatio(1.0),
  mTotalScaleRatio(1.0),
  mGeomCenterOffset(0.0),
  mSizeValue(0.0) {
  mCoordinate = handleNumber;
  mManipulator->highlightAxis(mManipulator->coordinate(mHandleNumber));
  mManipulator->setActive(true);
  mViewDistanceUnscaling = mViewpoint->viewDistanceUnscaling(selectedGeometry->matrix().translation());
  mSizeValue = mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate];
  const WbVector3 mouse3dPosition = computeLocalMousePosition(initialMousePosition);
  mMouseOffset = mouse3dPosition[mCoordinate] - mSizeValue;
  mViewpoint->lock();
}

WbDragResizeHandleEvent::~WbDragResizeHandleEvent() {
  mManipulator->setActive(false);
  mManipulator->showNormal();
  mManipulator->updateHandleDimensions(1.0f, 1.0f);

  mViewpoint->unlock();
}

WbVector3 WbDragResizeHandleEvent::computeLocalMousePosition(const QPoint &currentMousePosition) {
  const WbMatrix4 &matrix = mSelectedGeometry->matrix();

  WbMatrix3 unscaledMatrix = matrix.extracted3x3Matrix();
  const WbVector3 &scale = mSelectedGeometry->absoluteScale();
  unscaledMatrix.scale(1.0f / scale.x(), 1.0f / scale.y(), 1.0f / scale.z());

  WbVector3 attachedHandlePosition(mTotalScaleRatio * mViewDistanceUnscaling *
                                   mManipulator->relativeHandlePosition(mHandleNumber));
  attachedHandlePosition = unscaledMatrix * attachedHandlePosition;

  const float zEye = mViewpoint->zEye(attachedHandlePosition);
  WbVector3 localMousePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), zEye);
  localMousePosition = matrix.pseudoInversed(localMousePosition);
  localMousePosition /= scale;
  return localMousePosition;
}

void WbDragResizeHandleEvent::computeRatio(const QPoint &currentMousePosition) {
  WbVector3 localMousePosition = computeLocalMousePosition(currentMousePosition);
  const double newSizeValue = localMousePosition[mCoordinate] - mMouseOffset;
  mResizeRatio = newSizeValue / mSizeValue;

  if (abs(mResizeRatio) <= 0.01) {
    mResizeRatio = mResizeRatio < 0.0 ? -1.0 : 1.0;
    return;
  }

  mSizeValue = newSizeValue;
}

// Regular resize event: sphere, box, cylinder, capsule and cone

WbRegularResizeEvent::WbRegularResizeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                           WbGeometry *selectedGeometry) :
  WbDragResizeHandleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry) {
}

// Drag resizing the sphere

WbResizeSphereEvent::WbResizeSphereEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                         WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mSphere(static_cast<WbSphere *>(selectedGeometry)) {
}

void WbResizeSphereEvent::addActionInUndoStack() {
  WbUndoStack::instance()->push(new WbResizeCommand(mSphere, WbVector3(mTotalScaleRatio, mTotalScaleRatio, mTotalScaleRatio)));
}

void WbResizeSphereEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const double currentRadius = mSphere->radius() * mResizeRatio;

  if (exceedsFloatMax(currentRadius)) {
    emit aborted();
    return;
  }

  mSphere->setRadius(currentRadius);
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the cylinder

WbResizeCylinderEvent::WbResizeCylinderEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                             WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCylinder(static_cast<WbCylinder *>(selectedGeometry)) {
}

void WbResizeCylinderEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mCylinder, scale));
}

void WbResizeCylinderEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedRadius = mCylinder->radius() * mResizeRatio;

    if (exceedsFloatMax(resizedRadius)) {
      emit aborted();
      return;
    }

    mCylinder->setRadius(resizedRadius);
  } else {
    // Resizing the height
    const double resizedHeight = mCylinder->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCylinder->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the capsule

WbResizeCapsuleEvent::WbResizeCapsuleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                           WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCapsule(static_cast<WbCapsule *>(selectedGeometry)) {
}

void WbResizeCapsuleEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mCapsule, scale));
}

void WbResizeCapsuleEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedRadius = mCapsule->radius() * mResizeRatio;

    if (exceedsFloatMax(resizedRadius)) {
      emit aborted();
      return;
    }

    mCapsule->setRadius(resizedRadius);
  } else {
    // Resizing the height
    const float resizedHeight = mCapsule->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCapsule->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the box

WbResizeBoxEvent::WbResizeBoxEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                   WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mBox(static_cast<WbBox *>(selectedGeometry)) {
}

void WbResizeBoxEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mBox, scale));
}

void WbResizeBoxEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector3 &size = mBox->size();
  double currentValue;
  switch (mCoordinate) {
    case X:
      currentValue = size.x() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setX(currentValue);
      break;
    case Y:
      currentValue = size.y() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setY(currentValue);
      break;
    case Z:
      currentValue = size.z() * mResizeRatio;

      if (exceedsFloatMax(currentValue)) {
        emit aborted();
        return;
      }

      mBox->setZ(currentValue);
      break;
    default:
      assert(0);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the plane

WbResizePlaneEvent::WbResizePlaneEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                       WbGeometry *selectedGeometry) :
  WbDragResizeHandleEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mPlane(static_cast<WbPlane *>(selectedGeometry)) {
}

void WbResizePlaneEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mPlane, scale));
}

void WbResizePlaneEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  const WbVector2 &size = mPlane->size();
  if (mCoordinate == X) {
    double currentX = size.x() * mResizeRatio;

    if (exceedsFloatMax(currentX)) {
      emit aborted();
      return;
    }

    mPlane->setX(currentX);
  } else {
    double currentZ = size.y() * mResizeRatio;

    if (exceedsFloatMax(currentZ)) {
      emit aborted();
      return;
    }

    mPlane->setY(currentZ);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the cone

WbResizeConeEvent::WbResizeConeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                                     WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mCone(static_cast<WbCone *>(selectedGeometry)) {
}

void WbResizeConeEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mCone, scale));
}

void WbResizeConeEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  if (mCoordinate != Z) {
    // Resizing the radius
    const double resizedBottomRadius = mCone->bottomRadius() * mResizeRatio;

    if (exceedsFloatMax(resizedBottomRadius)) {
      emit aborted();
      return;
    }

    mCone->setBottomRadius(resizedBottomRadius);
  } else {
    // Resizing the height
    const double resizedHeight = mCone->height() * mResizeRatio;

    if (exceedsFloatMax(resizedHeight)) {
      emit aborted();
      return;
    }

    mCone->setHeight(resizedHeight);
  }
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the elevation grid

WbResizeElevationGridEvent::WbResizeElevationGridEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                       int handleNumber, WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mElevationGrid(static_cast<WbElevationGrid *>(selectedGeometry)) {
}

void WbResizeElevationGridEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mElevationGrid, scale));
}

void WbResizeElevationGridEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);
  switch (mCoordinate) {
    case X: {
      const double resizedXspacing = mElevationGrid->xSpacing() * mResizeRatio;

      if (exceedsFloatMax(resizedXspacing)) {
        emit aborted();
        return;
      }

      mElevationGrid->setXspacing(resizedXspacing);
      break;
    }
    case Y: {
      const double resizedYspacing = mElevationGrid->ySpacing() * mResizeRatio;
      if (exceedsFloatMax(resizedYspacing)) {
        emit aborted();
        return;
      }
      mElevationGrid->setYspacing(resizedYspacing);
      break;
    }
    case Z:
      if (exceedsFloatMax(mResizeRatio * mElevationGrid->heightRange())) {
        emit aborted();
        return;
      }
      mElevationGrid->setHeightScaleFactor(mResizeRatio);
      break;
    default:
      assert(false);
  }

  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}

// Drags resizing the indexed face set

WbResizeIndexedFaceSetEvent::WbResizeIndexedFaceSetEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                         int handleNumber, WbGeometry *selectedGeometry) :
  WbRegularResizeEvent(initialMousePosition, viewpoint, handleNumber, selectedGeometry),
  mIndexedFaceSet(static_cast<WbIndexedFaceSet *>(selectedGeometry)) {
  mSizeValue = mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate];
}

void WbResizeIndexedFaceSetEvent::addActionInUndoStack() {
  WbVector3 scale(1.0, 1.0, 1.0);
  scale[mCoordinate] = mTotalScaleRatio;
  WbUndoStack::instance()->push(new WbResizeCommand(mIndexedFaceSet, scale));
}

void WbResizeIndexedFaceSetEvent::apply(const QPoint &currentMousePosition) {
  computeRatio(currentMousePosition);

  if (exceedsFloatMax(mResizeRatio * mIndexedFaceSet->range(mCoordinate))) {
    emit aborted();
    return;
  }

  WbVector3 scale(1.0f, 1.0f, 1.0f);
  scale[mCoordinate] = mResizeRatio;
  mIndexedFaceSet->rescale(scale);

  // update global resize values
  mTotalScaleRatio *= mResizeRatio;
  mManipulator->updateHandleDimensions(mTotalScaleRatio, mViewDistanceUnscaling);
}
