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

#ifndef WB_DRAG_SOLID_EVENT_HPP
#define WB_DRAG_SOLID_EVENT_HPP

//
// Description: classes allowing to store data related with the solid mouse dragging
//

#include "WbAffinePlane.hpp"
#include "WbDragPoseEvent.hpp"
#include "WbRay.hpp"
#include "WbVector3.hpp"

#include <QtCore/QPointer>
#include <QtCore/QSize>

class WbPhysicsVectorRepresentation;
class WbWrenLabelOverlay;
class WbSolid;
class WbSolidMerger;
class WbViewpoint;

// Special translation and rotation drag event for Solid nodes that reset the physics
///////////////////////////////////////////////////////////////////////////////////////
class WbDragHorizontalSolidEvent : public WbDragHorizontalEvent {
public:
  WbDragHorizontalSolidEvent(const QPoint &initialPosition, WbViewpoint *viewpoint, WbSolid *selectedSolid);
  virtual ~WbDragHorizontalSolidEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  WbSolid *mSelectedSolid;
};

class WbDragVerticalSolidEvent : public WbDragVerticalEvent {
public:
  WbDragVerticalSolidEvent(const QPoint &initialPosition, WbViewpoint *viewpoint, WbSolid *selectedSolid);
  virtual ~WbDragVerticalSolidEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  WbSolid *mSelectedSolid;
};

class WbDragTranslateAlongAxisSolidEvent : public WbDragTranslateAlongAxisEvent {
  Q_OBJECT;

public:
  WbDragTranslateAlongAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize, WbViewpoint *viewpoint,
                                     int handleNumber, WbSolid *selectedSolid);
  virtual ~WbDragTranslateAlongAxisSolidEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  WbSolid *mSelectedSolid;
};

class WbDragRotateAroundWorldVerticalAxisSolidEvent : public WbDragRotateAroundWorldVerticalAxisEvent {
public:
  WbDragRotateAroundWorldVerticalAxisSolidEvent(const QPoint &initialPosition, WbViewpoint *viewpoint, WbSolid *selectedSolid);
  virtual ~WbDragRotateAroundWorldVerticalAxisSolidEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  WbSolid *mSelectedSolid;
};

class WbDragRotateAroundAxisSolidEvent : public WbDragRotateAroundAxisEvent {
  Q_OBJECT;

public:
  WbDragRotateAroundAxisSolidEvent(const QPoint &initialMousePosition, const QSize &widgetSize, WbViewpoint *viewpoint,
                                   int handleNumber, WbSolid *selectedSolid);
  virtual ~WbDragRotateAroundAxisSolidEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  WbSolid *mSelectedSolid;
};

// Abstract class for drag events involving physics changes only: user-defined forces and torques //
///////////////////////////////////////////////////////////////////////////////////////////////////

// WbDragPhysicsEvent class (abstract)
///////////////////////////////////////
class WbDragPhysicsEvent : public WbDragView3DEvent {
  Q_OBJECT;

public:
  WbDragPhysicsEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid);
  virtual ~WbDragPhysicsEvent();
  void apply(const QPoint &currentMousePosition) override;
  void lock();
  // Accessor
  virtual bool isLocked() const { return mIsLocked; }

signals:
  void aborted();  // triggers drag destruction in WbView3D

public slots:
  virtual void updateRenderingAndPhysics();

protected:
  void init();
  void applyChangesToWren();
  virtual void applyToOde() = 0;
  virtual void updateOrigin() = 0;
  virtual QString magnitudeString() const = 0;
  WbSolid *mSelectedSolid;
  WbPhysicsVectorRepresentation *mRepresentation;
  WbVector3 mOrigin;
  WbVector3 mEnd;
  WbVector3 mVector;
  double mScalingFactor;
  WbAffinePlane mDragPlane;
  WbRay mMouseRay;
  std::pair<bool, double> mIntersectionOutput;
  bool mIsLocked;
  WbWrenLabelOverlay *mTextOverlay;
  QSize mWidgetSize;
  float mViewDistanceScaling;
};

// WbDragForceEvent class
class WbDragForceEvent : public WbDragPhysicsEvent {
  Q_OBJECT;

public:
  WbDragForceEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid);

public slots:
  void applyToOde() override;

private:
  static const double FORCE_SCALING_FACTOR;
  WbVector3 mRelativeOrigin;
  void updateOrigin() override;
  QString magnitudeString() const override;
};

// WbDragTorqueEvent class
class WbDragTorqueEvent : public WbDragPhysicsEvent {
  Q_OBJECT;

public:
  WbDragTorqueEvent(const QSize &widgetSize, WbViewpoint *viewpoint, WbSolid *selectedSolid);

public slots:
  void applyToOde() override;

private:
  QPointer<WbSolidMerger> mSolidMerger;
  static const double TORQUE_SCALING_FACTOR;
  void updateOrigin() override;
  QString magnitudeString() const override;
};

#endif
