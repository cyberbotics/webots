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

#include "WbDragTransformEvent.hpp"

#include "WbAbstractTransform.hpp"
#include "WbEditCommand.hpp"
#include "WbMathsUtilities.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QDesktopWidget>

// WbDragTransformEvent constructor
WbDragTransformEvent::WbDragTransformEvent(WbViewpoint *viewpoint, WbAbstractTransform *selectedTransform) :
  WbDragKinematicsEvent(viewpoint),
  mSelectedTransform(selectedTransform) {
  mViewDistanceUnscaling = viewpoint->viewDistanceUnscaling(selectedTransform->position());
}

WbDragTransformEvent::~WbDragTransformEvent() {
  if (dynamic_cast<WbTransform *>(mViewpoint->followedSolid()) == mSelectedTransform)
    mViewpoint->updateFollowSolidState();
}

// WbTranslateEvent constructor
WbTranslateEvent::WbTranslateEvent(WbViewpoint *viewpoint, WbAbstractTransform *selectedTransform) :
  WbDragTransformEvent(viewpoint, selectedTransform),
  mInitialPosition(selectedTransform->translation()),
  mUpWorldVector(-WbWorld::instance()->worldInfo()->gravityUnitVector()),
  mMouseRay() {
  WbVector3 computedScaleFromParents = mSelectedTransform->absoluteScale();
  computedScaleFromParents /= mSelectedTransform->scale();
  mScaleFromParents = computedScaleFromParents;
}

WbTranslateEvent::~WbTranslateEvent() {
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedTransform->translationFieldValue(), WbVariant(mInitialPosition),
                                                  WbVariant(mSelectedTransform->translationFieldValue()->variantValue())));
}

//
// WbDragMatterEvent implemented classes
//

// Transform translation //
///////////////////////////

// WbDragHorizontalEvent functions
WbDragHorizontalEvent::WbDragHorizontalEvent(const QPoint &initialPosition, WbViewpoint *viewpoint,
                                             WbAbstractTransform *selectedTransform) :
  WbTranslateEvent(viewpoint, selectedTransform) {
  mDragPlane = WbAffinePlane(mUpWorldVector, mSelectedTransform->position());
  mViewpoint->viewpointRay(initialPosition.x(), initialPosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mTranslationOffset = mInitialPosition - mMouseRay.point(mIntersectionOutput.second);
  mViewpoint->lock();
}

WbDragHorizontalEvent::~WbDragHorizontalEvent() {
  mViewpoint->unlock();
}

void WbDragHorizontalEvent::apply(const QPoint &currentMousePosition) {
  mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
  mDragPlane.redefine(mUpWorldVector, mSelectedTransform->position());
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  WbVector3 displacementFromInitialPosition =
    mMouseRay.point(mIntersectionOutput.second) + mTranslationOffset - mInitialPosition;
  // remove any x or z scaling from parents (we shouldn't touch y as we're moving on the world horizontal plane)
  displacementFromInitialPosition.setX(displacementFromInitialPosition.x() / mScaleFromParents.x());
  displacementFromInitialPosition.setZ(displacementFromInitialPosition.z() / mScaleFromParents.z());
  mSelectedTransform->setTranslation(mInitialPosition + displacementFromInitialPosition);
  mSelectedTransform->emitTranslationOrRotationChangedByUser();
}

// WbDragVerticalEvent functions
WbDragVerticalEvent::WbDragVerticalEvent(const QPoint &initialPosition, WbViewpoint *viewpoint,
                                         WbAbstractTransform *selectedTransform) :
  WbTranslateEvent(viewpoint, selectedTransform),
  mNormal(viewpoint->orientation()->value().direction()) {
  // this event needs to use the actual position of the plane as we care about its depth from the Viewpoint
  mDragPlane = WbAffinePlane(mNormal, mSelectedTransform->position());
  mViewpoint->viewpointRay(initialPosition.x(), initialPosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mTranslationOffset = -mIntersectionOutput.second * mMouseRay.direction().dot(mUpWorldVector);
  mViewpoint->lock();
}

WbDragVerticalEvent::~WbDragVerticalEvent() {
  mViewpoint->unlock();
}

void WbDragVerticalEvent::apply(const QPoint &currentMousePosition) {
  mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
  mDragPlane.redefine(mNormal, mSelectedTransform->position());
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  const double verticalDrift = mIntersectionOutput.second * mMouseRay.direction().dot(mUpWorldVector) + mTranslationOffset;
  // divide by any y-axis scaling so that the overall translation applied to the node is local and independent of parent scale
  mSelectedTransform->setTranslation(mInitialPosition + mUpWorldVector * verticalDrift / mScaleFromParents.y());
  mSelectedTransform->emitTranslationOrRotationChangedByUser();
}

// WbDragTranslateAlongAxisEvent functions
WbDragTranslateAlongAxisEvent::WbDragTranslateAlongAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                             WbViewpoint *viewpoint, int handleNumber,
                                                             WbAbstractTransform *selectedTransform) :
  WbDragTransformEvent(viewpoint, selectedTransform),
  mInitialMatterPosition(selectedTransform->translation()),
  mTranslationOffset(0.0),
  mHandleNumber(handleNumber),
  mManipulator(selectedTransform->translateRotateManipulator()),
  mWidgetSizeFactor(1.0 / widgetSize.width(), 1.0 / widgetSize.height()),
  mStepSize(selectedTransform->translationStep()) {
  mCoordinate = mManipulator->coordinate(mHandleNumber);

  mManipulator->highlightAxis(mHandleNumber);
  mManipulator->setActive(true);

  // init translation offset label
  mTextOverlay = WbWrenLabelOverlay::createOrRetrieve(WbWrenLabelOverlay::dragCaptionOverlayId(),
                                                      WbStandardPaths::fontsPath() + "Arial.ttf");
  WbVector2 labelPosition = clampLabelPosition(initialMousePosition.x() / (float)(widgetSize.width()),
                                               initialMousePosition.y() / (float)(widgetSize.height()), mTextOverlay);
  mTextOverlay->setPosition(labelPosition.x(), labelPosition.y());
  mTextOverlay->setText("0.00 m");
  mTextOverlay->setColor(0x00FFFFFF);
  mTextOverlay->setBackgroundColor(0x70000000);
  mTextOverlay->setSize(0.1);
  mTextOverlay->applyChangesToWren();

  WbMatrix4 matrix(mSelectedTransform->matrix());
  const WbVector3 absoluteScale = matrix.scale();
  matrix.scale(1.0f / absoluteScale.x(), 1.0f / absoluteScale.y(), 1.0f / absoluteScale.z());

  // local offset
  WbVector3 attachedHandlePosition = matrix * (mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling);
  const double zEye = mViewpoint->zEye(attachedHandlePosition);
  WbVector3 mouse3dPosition = mViewpoint->pick(initialMousePosition.x(), initialMousePosition.y(), zEye);
  mouse3dPosition = matrix.pseudoInversed(mouse3dPosition);  // local position
  mHandleOffset = WbVector3(0.0, 0.0, 0.0);
  mHandleOffset[mCoordinate] = mouse3dPosition[mCoordinate];
  if (mStepSize > 0)
    mHandleOffset[mCoordinate] -= mStepSize * 0.5;
  mMouseOffset =
    (mViewDistanceUnscaling * mManipulator->relativeHandlePosition(mHandleNumber)[mCoordinate]) - mouse3dPosition[mCoordinate];

  WbVector2 objectScreenPosition;
  WbVector2 handleScreenPosition;
  mViewpoint->toPixels(mInitialMatterPosition, objectScreenPosition, attachedHandlePosition, handleScreenPosition);
  mDirectionOnScreen = handleScreenPosition - objectScreenPosition;

  mViewpoint->lock();
}

WbDragTranslateAlongAxisEvent::~WbDragTranslateAlongAxisEvent() {
  mManipulator->setActive(false);
  mManipulator->showNormal();

  // add translation in undo stack
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedTransform->translationFieldValue(),
                                                  WbVariant(mInitialMatterPosition),
                                                  WbVariant(mSelectedTransform->translationFieldValue()->variantValue())));

  // destroy translation offset label
  WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::dragCaptionOverlayId());

  mViewpoint->unlock();
}

void WbDragTranslateAlongAxisEvent::apply(const QPoint &currentMousePosition) {
  mViewDistanceUnscaling = mViewpoint->viewDistanceUnscaling(mSelectedTransform->position());

  WbMatrix4 matrix(mSelectedTransform->matrix());
  const WbVector3 absoluteScale = matrix.scale();
  matrix.scale(1.0f / absoluteScale.x(), 1.0f / absoluteScale.y(), 1.0f / absoluteScale.z());

  WbVector3 attachedHandlePosition = matrix * (mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling);
  const double zEye = mViewpoint->zEye(attachedHandlePosition);

  WbVector3 detachedHandlePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), zEye);
  detachedHandlePosition = matrix.pseudoInversed(detachedHandlePosition);  // local position

  WbVector3 difference = detachedHandlePosition - mHandleOffset;
  WbVector3 translationOffset;
  if (mStepSize <= 0)
    translationOffset[mCoordinate] = difference[mCoordinate];
  else
    translationOffset[mCoordinate] = floor(difference[mCoordinate] / mStepSize) * mStepSize;

  if (translationOffset[mCoordinate] != 0) {
    // convert local translation to parent transform coordinate system
    mTranslationOffset += translationOffset[mCoordinate];
    translationOffset = mSelectedTransform->rotation().toMatrix3() * translationOffset;
    mSelectedTransform->setTranslation(mSelectedTransform->translation() + translationOffset);
    mSelectedTransform->emitTranslationOrRotationChangedByUser();
  }

  // keep label near to drag detached handle
  WbVector2 objectScreenPosition;
  WbVector2 mousePosition(currentMousePosition.x(), currentMousePosition.y());
  mViewpoint->toPixels(matrix.translation(), objectScreenPosition);

  WbVector2 mousePositionOnScreen = mousePosition - objectScreenPosition;
  WbVector2 labelPosition =
    objectScreenPosition +
    (mousePositionOnScreen.dot(mDirectionOnScreen) / mDirectionOnScreen.dot(mDirectionOnScreen)) * mDirectionOnScreen;
  labelPosition =
    clampLabelPosition(labelPosition.x() * mWidgetSizeFactor.x(), labelPosition.y() * mWidgetSizeFactor.y(), mTextOverlay);

  mTextOverlay->moveToPosition(labelPosition.x(), labelPosition.y());
  if (mTranslationOffset > -1e-10 && mTranslationOffset < +1e-10)
    mTextOverlay->updateText("0.00 m");
  else {
    int left = mTranslationOffset;
    int right = abs(100 * (mTranslationOffset - left));
    mTextOverlay->updateText((mTranslationOffset < 0 ? "-" : "") + QString::number(abs(left)) +
                             QString(".%1").arg(right, 2, 10, QChar('0')) + " m");
  }
}

// Transform rotation //
////////////////////////

// WbDragRotateAroundWorldVerticalAxisEvent functions
WbDragRotateAroundWorldVerticalAxisEvent::WbDragRotateAroundWorldVerticalAxisEvent(const QPoint &initialMousePosition,
                                                                                   WbViewpoint *viewpoint,
                                                                                   WbAbstractTransform *selectedTransform) :
  WbDragTransformEvent(viewpoint, selectedTransform),
  mInitialQuaternionRotation(selectedTransform->rotation().toQuaternion()),
  mPreviousAngle(0.0),
  mInitialMouseXPosition(initialMousePosition.x()),
  mUpWorldVector(-WbWorld::instance()->worldInfo()->gravityUnitVector()) {
  mViewpoint->lock();
}

WbDragRotateAroundWorldVerticalAxisEvent::~WbDragRotateAroundWorldVerticalAxisEvent() {
  // add rotation in undo stack
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedTransform->rotationFieldValue(),
                                                  WbVariant(WbRotation(mInitialQuaternionRotation)),
                                                  WbVariant(mSelectedTransform->rotationFieldValue()->variantValue())));
  mViewpoint->unlock();
}

void WbDragRotateAroundWorldVerticalAxisEvent::apply(const QPoint &currentMousePosition) {
  // Horizontal movement rotates the object clockwise (and anti-clockwise)
  const QDesktopWidget *qDesktop = QApplication::desktop();
  const int screenNumber = qDesktop->screenNumber(QCursor::pos());
  // 4*pi used here so that a centered object can be fully rotated
  // by dragging all the way to the left or right of the screen
  double angle =
    4 * M_PI * ((currentMousePosition.x() - mInitialMouseXPosition) / qDesktop->screenGeometry(screenNumber).width());
  // add our new rotation
  WbQuaternion resultingRotation = WbQuaternion(mUpWorldVector, angle) * mInitialQuaternionRotation;
  mSelectedTransform->setRotation(WbRotation(resultingRotation));
  mSelectedTransform->emitTranslationOrRotationChangedByUser();

  mPreviousAngle = angle;
}

const double WbDragRotateAroundAxisEvent::RAD_TO_DEG = 180.0 / M_PI;

// WbDragRotateAroundAxisEvent functions
WbDragRotateAroundAxisEvent::WbDragRotateAroundAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                         WbViewpoint *viewpoint, int handleNumber,
                                                         WbAbstractTransform *selectedTransform) :
  WbDragTransformEvent(viewpoint, selectedTransform),
  mManipulator(selectedTransform->translateRotateManipulator()),
  mHandleNumber(handleNumber),
  mCoordinate(mManipulator->coordinate(handleNumber)),
  mInitialQuaternionRotation(selectedTransform->rotation().toQuaternion()),
  mInitialMatrix(mSelectedTransform->matrix()),
  mStepSize(selectedTransform->rotationStep()),
  mPreviousAngle(0.0) {
  mManipulator->highlightAxis(mHandleNumber + 3);
  mManipulator->setActive(true);

  const WbVector3 absoluteScale = mInitialMatrix.scale();
  mInitialMatrix.scale(1.0f / absoluteScale.x(), 1.0f / absoluteScale.y(), 1.0f / absoluteScale.z());

  WbVector4 scaledPos = mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling;
  WbVector4 handlePos = mInitialMatrix * scaledPos;
  mZEye = viewpoint->zEye(handlePos.toVector3());

  // init translation offset label
  mTextOverlay = WbWrenLabelOverlay::createOrRetrieve(WbWrenLabelOverlay::dragCaptionOverlayId(),
                                                      WbStandardPaths::fontsPath() + "Arial.ttf");
  mTextOverlay->setText("12/12\u03C0 rad");
  mTextOverlay->setColor(0x00FFFFFF);
  mTextOverlay->setBackgroundColor(0x70000000);
  mTextOverlay->setSize(0.1);
  mTextOverlay->applyChangesToWren();
  WbVector2 labelPosition = clampLabelPosition((25.0 + initialMousePosition.x()) / widgetSize.width(),
                                               (-20.0 + initialMousePosition.y()) / widgetSize.height(), mTextOverlay);
  mTextOverlay->moveToPosition(labelPosition.x(), labelPosition.y());

  // approximate step to M_PI fraction if possible;
  if (!WbMathsUtilities::computeRationalApproximation(mStepSize * M_1_PI, 100, mStepFractionNumerator,
                                                      mStepFractionDenominator)) {
    mStepFractionNumerator = mStepSize;
    mStepFractionDenominator = -1;
  }

  // compute initial rotation offset
  WbVector3 mousePosition = mViewpoint->pick(initialMousePosition.x(), initialMousePosition.y(), mZEye);
  mousePosition = mInitialMatrix.pseudoInversed(mousePosition);  // local position
  double x = mousePosition.dot(mManipulator->coordinateVector(mCoordinate + 1));
  double y = mousePosition.dot(mManipulator->coordinateVector(mCoordinate + 2));
  mInitialAngle = atan2(y, x);  // rotation angle

  mViewpoint->lock();
}

WbDragRotateAroundAxisEvent::~WbDragRotateAroundAxisEvent() {
  mManipulator->setActive(false);
  mManipulator->showNormal();

  // add rotation in undo stack
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedTransform->rotationFieldValue(),
                                                  WbVariant(WbRotation(mInitialQuaternionRotation)),
                                                  WbVariant(mSelectedTransform->rotationFieldValue()->variantValue())));

  // destroy translation offset label
  WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::dragCaptionOverlayId());

  mViewpoint->unlock();
}

void WbDragRotateAroundAxisEvent::apply(const QPoint &currentMousePosition) {
  WbVector3 detachedHandlePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), mZEye);
  detachedHandlePosition = mInitialMatrix.pseudoInversed(detachedHandlePosition);  // local position

  // project point on affine plane orthogonal to the rotation axis
  double x = detachedHandlePosition.dot(mManipulator->coordinateVector(mCoordinate + 1));
  double y = detachedHandlePosition.dot(mManipulator->coordinateVector(mCoordinate + 2));
  double angle = atan2(y, x) - mInitialAngle;  // rotation angle

  int stepCount = 0;
  if (mStepSize > 0) {
    if (angle < 0)
      angle += 2 * M_PI;
    stepCount = (int)(angle / mStepSize + 0.5);
    angle = stepCount * mStepSize;
  }

  // add new rotation
  WbQuaternion resultingRotation =
    mInitialQuaternionRotation * WbQuaternion(mManipulator->coordinateVector(mCoordinate), angle);
  mSelectedTransform->setRotation(WbRotation(resultingRotation));
  mSelectedTransform->emitTranslationOrRotationChangedByUser();

  // update label and handle rotation
  if (mStepFractionDenominator > 0)
    mTextOverlay->updateText(QString("%1/%2\u03C0 rad").arg(stepCount * mStepFractionNumerator).arg(mStepFractionDenominator));
  else
    mTextOverlay->updateText(QString("%1 rad").arg(angle));

  mPreviousAngle = angle;
}
