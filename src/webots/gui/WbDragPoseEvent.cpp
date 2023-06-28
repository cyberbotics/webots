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

#include "WbDragPoseEvent.hpp"

#include "WbAbstractPose.hpp"
#include "WbEditCommand.hpp"
#include "WbLog.hpp"
#include "WbMathsUtilities.hpp"
#include "WbSolid.hpp"
#include "WbStandardPaths.hpp"
#include "WbTransform.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtGui/QScreen>
#include <QtWidgets/QApplication>

#define DRAG_HORIZONTAL_MIN_COS 0.25

WbDragPoseEvent::WbDragPoseEvent(WbViewpoint *viewpoint, WbAbstractPose *selectedPose) :
  WbDragKinematicsEvent(viewpoint),
  mSelectedPose(selectedPose) {
  mViewDistanceUnscaling = viewpoint->viewDistanceUnscaling(selectedPose->position());
}

WbDragPoseEvent::~WbDragPoseEvent() {
  if (dynamic_cast<WbPose *>(mViewpoint->followedSolid()) == mSelectedPose)
    mViewpoint->updateFollowSolidState();
}

WbTranslateEvent::WbTranslateEvent(WbViewpoint *viewpoint, WbAbstractPose *selectedPose) :
  WbDragPoseEvent(viewpoint, selectedPose),
  mInitialPosition(selectedPose->translation()),
  mUpWorldVector(WbWorld::instance()->worldInfo()->upVector()),
  mMouseRay() {
  WbVector3 computedScaleFromParents;
  const WbPose *pose = dynamic_cast<const WbPose *>(mSelectedPose);
  if (pose) {
    const WbTransform *pt = pose->upperTransform();
    if (pt)
      computedScaleFromParents = pt->absoluteScale();
    else
      computedScaleFromParents = WbVector3(1, 1, 1);
  } else
    computedScaleFromParents = WbVector3(1, 1, 1);
  const WbTransform *t = dynamic_cast<const WbTransform *>(mSelectedPose);
  if (t)
    computedScaleFromParents /= t->scale();

  mScaleFromParents = computedScaleFromParents;
}

WbTranslateEvent::~WbTranslateEvent() {
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedPose->translationFieldValue(), WbVariant(mInitialPosition),
                                                  WbVariant(mSelectedPose->translationFieldValue()->variantValue())));
}

//
// WbDragMatterEvent implemented classes
//

// Transform translation //
///////////////////////////

// WbDragHorizontalEvent functions
WbDragHorizontalEvent::WbDragHorizontalEvent(const QPoint &initialPosition, WbViewpoint *viewpoint,
                                             WbAbstractPose *selectedPose) :
  WbTranslateEvent(viewpoint, selectedPose) {
  mDragPlane = WbAffinePlane(mUpWorldVector, mSelectedPose->position());
  mViewpoint->viewpointRay(initialPosition.x(), initialPosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mTranslationOffset = mMouseRay.point(mIntersectionOutput.second);
  mViewpoint->lock();

  // event occurs only if the mouse ray is not parallel to the horizontal drag plane
  WbRay normalizedMouseRay = mMouseRay;
  normalizedMouseRay.normalize();
  if (abs(normalizedMouseRay.direction().dot(mUpWorldVector)) > DRAG_HORIZONTAL_MIN_COS) {
    mIsMouseRayValid = true;
    assert(mSelectedPose->baseNode()->upperPose() == NULL);  // is top Pose node
  } else {
    mIsMouseRayValid = false;
    WbLog::warning(tr("To drag this element, first rotate the view so that the horizontal plane is clearly visible."));
  }
}

WbDragHorizontalEvent::~WbDragHorizontalEvent() {
  mViewpoint->unlock();
}

void WbDragHorizontalEvent::apply(const QPoint &currentMousePosition) {
  if (mIsMouseRayValid) {
    mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
    mDragPlane.redefine(mUpWorldVector, mSelectedPose->position());
    mIntersectionOutput = mMouseRay.intersects(mDragPlane);
    WbVector3 displacementFromInitialPosition = mMouseRay.point(mIntersectionOutput.second) - mTranslationOffset;
    // remove any x or z scaling from parents (we shouldn't touch y as we're moving on the world horizontal plane)
    displacementFromInitialPosition.setX(displacementFromInitialPosition.x() / mScaleFromParents.x());
    displacementFromInitialPosition.setZ(displacementFromInitialPosition.z() / mScaleFromParents.z());

    mSelectedPose->setTranslation((mInitialPosition + displacementFromInitialPosition).rounded(WbPrecision::GUI_MEDIUM));
    mSelectedPose->emitTranslationOrRotationChangedByUser();
  }
}

// WbDragVerticalEvent functions
WbDragVerticalEvent::WbDragVerticalEvent(const QPoint &initialPosition, WbViewpoint *viewpoint, WbAbstractPose *selectedPose) :
  WbTranslateEvent(viewpoint, selectedPose),
  mNormal(viewpoint->orientation()->value().direction()) {
  // this event needs to use the actual position of the plane as we care about its depth from the Viewpoint
  mDragPlane = WbAffinePlane(mNormal, mSelectedPose->position());
  mViewpoint->viewpointRay(initialPosition.x(), initialPosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mTranslationOffset = mMouseRay.point(mIntersectionOutput.second);
  mViewpoint->lock();
}

WbDragVerticalEvent::~WbDragVerticalEvent() {
  mViewpoint->unlock();
}

void WbDragVerticalEvent::apply(const QPoint &currentMousePosition) {
  mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
  mDragPlane.redefine(mNormal, mSelectedPose->position());
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  const WbVector3 displacementFromInitialPosition(mMouseRay.point(mIntersectionOutput.second) - mTranslationOffset);
  // divide by any y-axis scaling so that the overall translation applied to the node is local and independent of parent scale
  mSelectedPose->setTranslation(
    (mInitialPosition + displacementFromInitialPosition * mUpWorldVector).rounded(WbPrecision::GUI_MEDIUM));
  mSelectedPose->emitTranslationOrRotationChangedByUser();
}

// WbDragTranslateAlongAxisEvent functions
WbDragTranslateAlongAxisEvent::WbDragTranslateAlongAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                             WbViewpoint *viewpoint, int handleNumber,
                                                             WbAbstractPose *selectedPose) :
  WbDragPoseEvent(viewpoint, selectedPose),
  mInitialMatterPosition(selectedPose->translation()),
  mTranslationOffset(0.0),
  mHandleNumber(handleNumber),
  mManipulator(selectedPose->translateRotateManipulator()),
  mWidgetSizeFactor(1.0 / widgetSize.width(), 1.0 / widgetSize.height()),
  mStepSize(selectedPose->translationStep()),
  mAbsoluteScale(1.0) {
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

  WbMatrix4 matrix(mSelectedPose->matrix());
  const WbVector3 &scale = matrix.scale();
  matrix.scale(1.0f / scale.x(), 1.0f / scale.y(), 1.0f / scale.z());
  const WbTransform *pt = mSelectedPose->baseNode()->upperTransform();
  if (pt)
    mAbsoluteScale = pt->absoluteScale()[mCoordinate];

  // local offset
  const WbVector3 attachedHandlePosition(matrix *
                                         (mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling));
  const double zEye = mViewpoint->zEye(attachedHandlePosition);
  WbVector3 mouse3dPosition = mViewpoint->pick(initialMousePosition.x(), initialMousePosition.y(), zEye);
  mouse3dPosition = matrix.pseudoInversed(mouse3dPosition);  // local position
  mHandleOffset = WbVector3(0.0, 0.0, 0.0);
  mHandleOffset[mCoordinate] = mouse3dPosition[mCoordinate];
  if (mStepSize > 0)
    mHandleOffset[mCoordinate] -= mStepSize * 0.5 * mAbsoluteScale;
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
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedPose->translationFieldValue(), WbVariant(mInitialMatterPosition),
                                                  WbVariant(mSelectedPose->translationFieldValue()->variantValue())));

  // destroy translation offset label
  WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::dragCaptionOverlayId());

  mViewpoint->unlock();
}

void WbDragTranslateAlongAxisEvent::apply(const QPoint &currentMousePosition) {
  mViewDistanceUnscaling = mViewpoint->viewDistanceUnscaling(mSelectedPose->position());

  WbMatrix4 matrix(mSelectedPose->matrix());
  const WbVector3 &scale = matrix.scale();
  matrix.scale(1.0f / scale.x(), 1.0f / scale.y(), 1.0f / scale.z());

  WbVector3 attachedHandlePosition = matrix * (mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling);
  const double zEye = mViewpoint->zEye(attachedHandlePosition);

  WbVector3 detachedHandlePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), zEye);
  detachedHandlePosition = matrix.pseudoInversed(detachedHandlePosition);  // local position

  const double difference = (detachedHandlePosition[mCoordinate] - mHandleOffset[mCoordinate]) / mAbsoluteScale;
  WbVector3 translationOffset;
  if (mStepSize <= 0)
    translationOffset[mCoordinate] = difference;
  else
    translationOffset[mCoordinate] = floor(difference / mStepSize) * mStepSize;

  if (translationOffset[mCoordinate] != 0) {
    // convert local translation to parent transform coordinate system
    mTranslationOffset += translationOffset[mCoordinate];
    translationOffset = mSelectedPose->rotation().toMatrix3() * translationOffset;
    mSelectedPose->setTranslation((mSelectedPose->translation() + translationOffset).rounded(WbPrecision::GUI_MEDIUM));
    mSelectedPose->emitTranslationOrRotationChangedByUser();
  }

  // keep label near to drag detached handle
  WbVector2 objectScreenPosition;
  const WbVector2 mousePosition(currentMousePosition.x(), currentMousePosition.y());
  mViewpoint->toPixels(matrix.translation(), objectScreenPosition);

  const WbVector2 mousePositionOnScreen(mousePosition - objectScreenPosition);
  WbVector2 labelPosition =
    objectScreenPosition +
    (mousePositionOnScreen.dot(mDirectionOnScreen) / mDirectionOnScreen.dot(mDirectionOnScreen)) * mDirectionOnScreen;
  labelPosition =
    clampLabelPosition(labelPosition.x() * mWidgetSizeFactor.x(), labelPosition.y() * mWidgetSizeFactor.y(), mTextOverlay);

  mTextOverlay->moveToPosition(labelPosition.x(), labelPosition.y());
  if (mTranslationOffset > -1e-10 && mTranslationOffset < +1e-10)
    mTextOverlay->updateText("0.00 m");
  else {
    const int left = mTranslationOffset;
    const int right = abs(100 * (mTranslationOffset - left));
    mTextOverlay->updateText((mTranslationOffset < 0 ? "-" : "") + QString::number(abs(left)) +
                             QString(".%1").arg(right, 2, 10, QChar('0')) + " m");
  }
}

// Transform rotation //
////////////////////////

// WbDragRotateAroundWorldVerticalAxisEvent functions
WbDragRotateAroundWorldVerticalAxisEvent::WbDragRotateAroundWorldVerticalAxisEvent(const QPoint &initialMousePosition,
                                                                                   WbViewpoint *viewpoint,
                                                                                   WbAbstractPose *selectedPose) :
  WbDragPoseEvent(viewpoint, selectedPose),
  mInitialQuaternionRotation(selectedPose->rotation().toQuaternion()),
  mPreviousAngle(0.0),
  mInitialMouseXPosition(initialMousePosition.x()),
  mUpWorldVector(WbWorld::instance()->worldInfo()->upVector()) {
  mViewpoint->lock();
}

WbDragRotateAroundWorldVerticalAxisEvent::~WbDragRotateAroundWorldVerticalAxisEvent() {
  // add rotation in undo stack
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedPose->rotationFieldValue(),
                                                  WbVariant(WbRotation(mInitialQuaternionRotation)),
                                                  WbVariant(mSelectedPose->rotationFieldValue()->variantValue())));
  mViewpoint->unlock();
}

void WbDragRotateAroundWorldVerticalAxisEvent::apply(const QPoint &currentMousePosition) {
  // Horizontal movement rotates the object clockwise (and anti-clockwise)
  const QScreen *screen = QGuiApplication::screenAt(QCursor::pos());
  // 4*pi used here so that a centered object can be fully rotated
  // by dragging all the way to the left or right of the screen
  const double angle = 4 * M_PI * ((currentMousePosition.x() - mInitialMouseXPosition) / screen->geometry().width());
  // add our new rotation
  const WbQuaternion resultingRotation = WbQuaternion(mUpWorldVector, angle) * mInitialQuaternionRotation;
  mSelectedPose->setRotation(WbRotation(resultingRotation).rounded(WbPrecision::GUI_MEDIUM));
  mSelectedPose->emitTranslationOrRotationChangedByUser();

  mPreviousAngle = angle;
}

const double WbDragRotateAroundAxisEvent::RAD_TO_DEG = 180.0 / M_PI;

// WbDragRotateAroundAxisEvent functions
WbDragRotateAroundAxisEvent::WbDragRotateAroundAxisEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                         WbViewpoint *viewpoint, int handleNumber,
                                                         WbAbstractPose *selectedPose) :
  WbDragPoseEvent(viewpoint, selectedPose),
  mManipulator(selectedPose->translateRotateManipulator()),
  mHandleNumber(handleNumber),
  mCoordinate(mManipulator->coordinate(handleNumber)),
  mInitialQuaternionRotation(selectedPose->rotation().toQuaternion()),
  mInitialMatrix(mSelectedPose->matrix()),
  mInitialPosition(mSelectedPose->position()),
  mStepSize(selectedPose->rotationStep()),
  mPreviousAngle(0.0),
  mInitialAngle(NAN) {
  mManipulator->highlightAxis(mHandleNumber + 3);
  mManipulator->setActive(true);

  const WbTransform *const ut = dynamic_cast<const WbTransform *const>(selectedPose);
  if (ut)
    mInitialMatrix.scale(1.0f / ut->scale().x(), 1.0f / ut->scale().y(), 1.0f / ut->scale().z());

  WbVector4 scaledPos(mManipulator->relativeHandlePosition(mHandleNumber) * mViewDistanceUnscaling);
  WbVector4 handlePos = mInitialMatrix * scaledPos;
  mZEye = viewpoint->zEye(handlePos.toVector3());

  viewpoint->toPixels(selectedPose->position(), mObjectScreenPosition);

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
  mViewpoint->lock();
}

WbDragRotateAroundAxisEvent::~WbDragRotateAroundAxisEvent() {
  mManipulator->setActive(false);
  mManipulator->showRotationLine(false);
  mManipulator->showNormal();

  // add rotation in undo stack
  WbUndoStack::instance()->push(new WbEditCommand(mSelectedPose->rotationFieldValue(),
                                                  WbVariant(WbRotation(mInitialQuaternionRotation)),
                                                  WbVariant(mSelectedPose->rotationFieldValue()->variantValue())));

  // destroy translation offset label
  WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::dragCaptionOverlayId());

  mViewpoint->unlock();
}

void WbDragRotateAroundAxisEvent::apply(const QPoint &currentMousePosition) {
  WbVector3 detachedHandlePosition = mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), mZEye);
  detachedHandlePosition = mInitialMatrix.pseudoInversed(detachedHandlePosition);  // local position

  // Depending on the rotation vector direction in respect to the pointview direction of rotation should adapted
  const int sign = WbMatrix3(mInitialQuaternionRotation)
                         .column(mCoordinate)
                         .dot((mInitialPosition - mViewpoint->position()->value()).normalized()) > 0 ?
                     1 :
                     -1;
  if (isnan(mInitialAngle)) {
    const double distance = sqrt(pow(mObjectScreenPosition.x() - currentMousePosition.x(), 2) +
                                 pow(mObjectScreenPosition.y() - currentMousePosition.y(), 2));
    if (distance < 8)
      return;
    mInitialAngle =
      sign * atan2(mObjectScreenPosition.y() - currentMousePosition.y(), mObjectScreenPosition.x() - currentMousePosition.x());
  }
  double angle =
    sign * atan2(mObjectScreenPosition.y() - currentMousePosition.y(), mObjectScreenPosition.x() - currentMousePosition.x()) -
    mInitialAngle;  // rotation angle

  mManipulator->showRotationLine(true);
  mManipulator->updateRotationLine(mViewpoint->pick(mObjectScreenPosition.x(), mObjectScreenPosition.y(), mZEye),
                                   mViewpoint->pick(currentMousePosition.x(), currentMousePosition.y(), mZEye),
                                   mViewpoint->orientation()->value(), mViewDistanceUnscaling);

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
  mSelectedPose->setRotation(WbRotation(resultingRotation).rounded(WbPrecision::GUI_MEDIUM));
  mSelectedPose->emitTranslationOrRotationChangedByUser();

  // update label and handle rotation
  if (mStepFractionDenominator > 0)
    mTextOverlay->updateText(QString("%1/%2\u03C0 rad").arg(stepCount * mStepFractionNumerator).arg(mStepFractionDenominator));
  else
    mTextOverlay->updateText(QString("%1 rad").arg(angle));

  mPreviousAngle = angle;
}
