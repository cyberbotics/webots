// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbDragViewpointEvent.hpp"

#include "WbQuaternion.hpp"
#include "WbSFRotation.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenRenderingContext.hpp"

#include <wren/camera.h>

// WbDragViewpointEvent constructor
WbDragViewpointEvent::WbDragViewpointEvent(WbViewpoint *viewpoint) : WbDragKinematicsEvent(viewpoint) {
}

// Translate viewpoint //
/////////////////////////

// WbTranslateViewpointEvent functions

WbTranslateViewpointEvent::WbTranslateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, double scale) :
  WbDragViewpointEvent(viewpoint),
  mInitialMousePosition(initialMousePosition),
  mDifference(),
  mInitialCameraPosition(viewpoint->position()->value()),
  mScaleFactor(scale) {
}

WbTranslateViewpointEvent::~WbTranslateViewpointEvent() {
  if (mInitialCameraPosition != mViewpoint->position()->value())
    WbWorld::instance()->setModified();
}

void WbTranslateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;
  mDifference = currentMousePosition - mInitialMousePosition;
  const double targetRight = -mScaleFactor * mDifference.x();
  const double targetUp = mScaleFactor * mDifference.y();
  const WbRotation &orientation = mViewpoint->orientation()->value();
  const WbVector3 target = targetRight * orientation.right() + targetUp * orientation.up();
  mViewpoint->position()->setValue(mInitialCameraPosition + target);
}

// Rotate Viewpoint //
//////////////////////

// WbRotateViewpointEvent functions

WbRotateViewpointEvent::WbRotateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, bool objectPicked) :
  WbDragViewpointEvent(viewpoint),
  mPreviousMousePosition(initialMousePosition),
  mDelta(),
  mWorldUpVector(-WbWorld::instance()->worldInfo()->gravityUnitVector()),
  mIsObjectPicked(objectPicked) {
  mViewpoint->lockRotationCenter();
}

WbRotateViewpointEvent::~WbRotateViewpointEvent() {
  mViewpoint->unlockRotationCenter();
  if (!mDelta.isNull())
    WbWorld::instance()->setModified();
}

void WbRotateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;

  mDelta = currentMousePosition - mPreviousMousePosition;
  mPreviousMousePosition = currentMousePosition;
  double halfPitchAngle = -0.005 * mDelta.y();
  double halfYawAngle = -0.005 * mDelta.x();
  if (!mIsObjectPicked) {
    halfPitchAngle /= -8;
    halfYawAngle /= -8;
  }
  const double sinusYaw = sin(halfYawAngle);
  const double sinusPitch = sin(halfPitchAngle);
  WbSFRotation *orientation = mViewpoint->orientation();
  WbSFVector3 *position = mViewpoint->position();
  const WbRotation &orientationValue = orientation->value();
  const WbVector3 pitch = orientationValue.right();
  const WbQuaternion pitchRotation(cos(halfPitchAngle), sinusPitch * pitch.x(), sinusPitch * pitch.y(), sinusPitch * pitch.z());
  const WbQuaternion yawRotation(cos(halfYawAngle), sinusYaw * mWorldUpVector.x(), sinusYaw * mWorldUpVector.y(),
                                 sinusYaw * mWorldUpVector.z());
  // Updates camera's position and orientation
  const WbQuaternion deltaRotation = yawRotation * pitchRotation;
  const WbVector3 &rotationCenter = mViewpoint->rotationCenter();  // picked coordinates or viewpoint position
  const WbVector3 currentPosition = deltaRotation * (position->value() - rotationCenter) + rotationCenter;
  const WbQuaternion currentOrientation = deltaRotation * orientationValue.toQuaternion();
  position->setValue(currentPosition);
  orientation->setValue(WbRotation(currentOrientation));
}

// Zoom and rotate Viewpoint //
///////////////////////////////

// WbZoomAndRotateViewpointEvent functions

WbZoomAndRotateViewpointEvent::WbZoomAndRotateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint,
                                                             double scale) :
  WbDragViewpointEvent(viewpoint),
  mPreviousMousePosition(initialMousePosition),
  mDelta(),
  mZscaleFactor(scale),
  mProjectionModeIsOrthographic(viewpoint->projectionMode() == WR_CAMERA_PROJECTION_MODE_ORTHOGRAPHIC) {
}

WbZoomAndRotateViewpointEvent::~WbZoomAndRotateViewpointEvent() {
  if (!mDelta.isNull())
    WbWorld::instance()->setModified();
}

void WbZoomAndRotateViewpointEvent::apply(const QPoint &currentMousePosition) {
  if (mViewpoint->isLocked())
    return;

  mDelta = currentMousePosition - mPreviousMousePosition;
  mPreviousMousePosition = currentMousePosition;
  if (mProjectionModeIsOrthographic) {
    if (mDelta.y() > 0.0)
      mViewpoint->incOrthographicViewHeight();
    else
      mViewpoint->decOrthographicViewHeight();
  }
  WbSFVector3 *position = mViewpoint->position();
  WbSFRotation *orientation = mViewpoint->orientation();
  const WbRotation &orientationValue = orientation->value();
  const WbVector3 rollVector = orientationValue.direction();
  const WbVector3 zDisplacement = (mZscaleFactor * mDelta.y()) * rollVector;
  const WbQuaternion roll(rollVector, 0.01 * mDelta.x());
  position->setValue(position->value() + zDisplacement);
  orientation->setValue(WbRotation(roll * orientationValue.toQuaternion()));
}
