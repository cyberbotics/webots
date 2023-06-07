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

#include "WbDragSolidEvent.hpp"

#include "WbEditCommand.hpp"
#include "WbLog.hpp"
#include "WbMathsUtilities.hpp"
#include "WbPhysicsVectorRepresentation.hpp"
#include "WbRotation.hpp"
#include "WbSimulationWorld.hpp"
#include "WbSolid.hpp"
#include "WbSolidMerger.hpp"
#include "WbStandardPaths.hpp"
#include "WbTranslateRotateManipulator.hpp"
#include "WbUndoStack.hpp"
#include "WbViewpoint.hpp"
#include "WbWorld.hpp"
#include "WbWrenLabelOverlay.hpp"
#include "WbWrenRenderingContext.hpp"

#include <QtCore/QSize>

WbDragHorizontalSolidEvent::WbDragHorizontalSolidEvent(const QPoint &initialPosition, WbViewpoint *viewpoint,
                                                       WbSolid *selectedSolid) :
  WbDragHorizontalEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

WbDragHorizontalSolidEvent::~WbDragHorizontalSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void WbDragHorizontalSolidEvent::apply(const QPoint &currentMousePosition) {
  WbDragHorizontalEvent::apply(currentMousePosition);
}

WbDragVerticalSolidEvent::WbDragVerticalSolidEvent(const QPoint &initialPosition, WbViewpoint *viewpoint,
                                                   WbSolid *selectedSolid) :
  WbDragVerticalEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

WbDragVerticalSolidEvent::~WbDragVerticalSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void WbDragVerticalSolidEvent::apply(const QPoint &currentMousePosition) {
  WbDragVerticalEvent::apply(currentMousePosition);
}

WbDragTranslateAlongAxisSolidEvent::WbDragTranslateAlongAxisSolidEvent(const QPoint &initialMousePosition,
                                                                       const QSize &widgetSize, WbViewpoint *viewpoint,
                                                                       int handleNumber, WbSolid *selectedSolid) :
  WbDragTranslateAlongAxisEvent(initialMousePosition, widgetSize, viewpoint, handleNumber, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

WbDragTranslateAlongAxisSolidEvent::~WbDragTranslateAlongAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void WbDragTranslateAlongAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  WbDragTranslateAlongAxisEvent::apply(currentMousePosition);
}

WbDragRotateAroundWorldVerticalAxisSolidEvent::WbDragRotateAroundWorldVerticalAxisSolidEvent(const QPoint &initialPosition,
                                                                                             WbViewpoint *viewpoint,
                                                                                             WbSolid *selectedSolid) :
  WbDragRotateAroundWorldVerticalAxisEvent(initialPosition, viewpoint, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

WbDragRotateAroundWorldVerticalAxisSolidEvent::~WbDragRotateAroundWorldVerticalAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void WbDragRotateAroundWorldVerticalAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  WbDragRotateAroundWorldVerticalAxisEvent::apply(currentMousePosition);
}

WbDragRotateAroundAxisSolidEvent::WbDragRotateAroundAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize,
                                                                   WbViewpoint *viewpoint, int handleNumber,
                                                                   WbSolid *selectedSolid) :
  WbDragRotateAroundAxisEvent(initialMousePosition, widgetSize, viewpoint, handleNumber, selectedSolid),
  mSelectedSolid(selectedSolid) {
  mSelectedSolid->pausePhysics();
}

WbDragRotateAroundAxisSolidEvent::~WbDragRotateAroundAxisSolidEvent() {
  mSelectedSolid->resumePhysics();
}

void WbDragRotateAroundAxisSolidEvent::apply(const QPoint &currentMousePosition) {
  WbDragRotateAroundAxisEvent::apply(currentMousePosition);
}

// Physics drag events //
/////////////////////////

// Abstract class

// WbDragPhysicsEvent constructor
WbDragPhysicsEvent::WbDragPhysicsEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid) :
  WbDragView3DEvent(viewpoint),
  mSelectedSolid(selectedSolid),
  mIsLocked(false),
  mWidgetSize(widgetSize),
  mViewDistanceScaling(1.0f) {
  // init label
  mTextOverlay = WbWrenLabelOverlay::createOrRetrieve(WbWrenLabelOverlay::dragCaptionOverlayId(),
                                                      WbStandardPaths::fontsPath() + "Arial.ttf");
  mTextOverlay->setColor(0x00FFFFFF);
  mTextOverlay->setBackgroundColor(0x70000000);
  mTextOverlay->setSize(0.1);
  mTextOverlay->setText("0");
  mTextOverlay->applyChangesToWren();

  connect(WbSimulationWorld::instance(), &WbSimulationWorld::physicsStepStarted, this,
          &WbDragPhysicsEvent::updateRenderingAndPhysics, Qt::UniqueConnection);
  mViewpoint->lock();
}

// WbDragPhysicsEvent destructor
WbDragPhysicsEvent::~WbDragPhysicsEvent() {
  delete mRepresentation;
  // destroy translation offset label
  WbWrenLabelOverlay::removeLabel(WbWrenLabelOverlay::dragCaptionOverlayId());
  mViewpoint->unlock();
}

void WbDragPhysicsEvent::init() {
  // Set size so that the arrow head has a length of 50px
  const float screenSize = 50.0f;  // px
  mViewDistanceScaling = mViewpoint->viewDistanceUnscaling(mSelectedSolid->position()) * screenSize;
}

void WbDragPhysicsEvent::lock() {
  mIsLocked = true;
  disconnect(WbSimulationWorld::instance(), &WbSimulationWorld::physicsStepStarted, this,
             &WbDragPhysicsEvent::updateRenderingAndPhysics);
}

void WbDragPhysicsEvent::apply(const QPoint &currentMousePosition) {
  // If the mouse button was released in PAUSE mode, skip updates
  if (mIsLocked)
    return;

  // Updates the World Coordinates of Force or the Torque's end
  mViewpoint->viewpointRay(currentMousePosition.x(), currentMousePosition.y(), mMouseRay);
  mIntersectionOutput = mMouseRay.intersects(mDragPlane);
  mEnd = mMouseRay.point(mIntersectionOutput.second);

  // Updates the World Coordinates of Force or Torque's origin
  updateOrigin();

  if (exceedsFloatMax(mEnd) || exceedsFloatMax(mVector) || exceedsFloatMax(mOrigin)) {
    emit aborted();
    return;
  }

  // Updates graphical representation
  applyChangesToWren();

  // display the magnitude of the generated vector in the overlay label
  const WbVector2 labelPosition = clampLabelPosition((25.0 + currentMousePosition.x()) / mWidgetSize.width(),
                                                     (-20.0 + currentMousePosition.y()) / mWidgetSize.height(), mTextOverlay);
  mTextOverlay->moveToPosition(labelPosition.x(), labelPosition.y());
  mTextOverlay->updateText(magnitudeString());
}

void WbDragPhysicsEvent::applyChangesToWren() {
  mRepresentation->updatePosition(mOrigin, mEnd, mViewpoint->orientation()->value());
}

void WbDragPhysicsEvent::updateRenderingAndPhysics() {
  updateOrigin();

  if (exceedsFloatMax(mEnd) || exceedsFloatMax(mVector) || exceedsFloatMax(mOrigin)) {
    emit aborted();
    return;
  }

  mTextOverlay->updateText(magnitudeString());
  applyChangesToWren();
  applyToOde();
}

//
// WbPhysicDragEvent implemented functions
//

// Add a force by dragging the mouse  //
////////////////////////////////////////

// WbDragForceEvent functions

WbDragForceEvent::WbDragForceEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid) :
  WbDragPhysicsEvent(widgetSize, viewpoint, selectedSolid) {
  mRepresentation = new WbForceRepresentation();
  mOrigin = viewpoint->rotationCenter();
  mRelativeOrigin = selectedSolid->matrix().pseudoInversed(mOrigin);
  mEnd = mOrigin;
  mDragPlane = WbAffinePlane(viewpoint->orientation()->value().direction(), mOrigin);
  mScalingFactor = WbWorld::instance()->worldInfo()->dragForceScale() * selectedSolid->mass();
  init();
  mRepresentation->setScale(mViewDistanceScaling);
}

void WbDragForceEvent::updateOrigin() {
  mOrigin = mSelectedSolid->matrix() * mRelativeOrigin;
  mVector = mEnd - mOrigin;
}

void WbDragForceEvent::applyToOde() {
  // ODE
  mSelectedSolid->awake();
  mSelectedSolid->addForceAtPosition(mScalingFactor * mVector.length2() * mVector, mOrigin);
}

QString WbDragForceEvent::magnitudeString() const {
  return WbPrecision::doubleToString(mScalingFactor * mVector.length2() * mVector.length(), WbPrecision::GUI_MEDIUM) + " N";
}

// Add a torque by dragging the mouse  //
/////////////////////////////////////////

// WbDragTorqueEvent functions

WbDragTorqueEvent::WbDragTorqueEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid) :
  WbDragPhysicsEvent(widgetSize, viewpoint, selectedSolid->solidMerger()->solid()),
  mSolidMerger(selectedSolid->solidMerger()) {
  mRepresentation = new WbTorqueRepresentation();
  mOrigin = mSelectedSolid->matrix() * mSolidMerger->centerOfMass();
  mEnd = mOrigin;
  mDragPlane = WbAffinePlane(viewpoint->orientation()->value().direction(), mOrigin);
  mScalingFactor = WbWorld::instance()->worldInfo()->dragTorqueScale() * selectedSolid->mass();
  init();
  mRepresentation->setScale(mViewDistanceScaling);
}

void WbDragTorqueEvent::updateOrigin() {
  mOrigin = mSelectedSolid->matrix() * mSolidMerger->centerOfMass();
  mVector = mEnd - mOrigin;
}

void WbDragTorqueEvent::applyToOde() {
  // ODE
  mSelectedSolid->awake();
  mSelectedSolid->addTorque(mScalingFactor * mVector.length2() * mVector);
}

QString WbDragTorqueEvent::magnitudeString() const {
  return WbPrecision::doubleToString(mScalingFactor * mVector.length2() * mVector.length(), WbPrecision::GUI_MEDIUM) + " Nm";
}
