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

#ifndef WB_DRAG_RESIZE_EVENT_HPP
#define WB_DRAG_RESIZE_EVENT_HPP

//
// Description: classes allowing to store data related with resize mouse dragging
//
// This class allows geometries fields to be modified individually,
// whereas WbDragScaleEvent scales all the fields at the same time (uniform scale).
//

#include "WbAbstractDragEvent.hpp"

#include <QtCore/QPoint>

class WbBox;
class WbCapsule;
class WbCone;
class WbCylinder;
class WbElevationGrid;
class WbIndexedFaceSet;
class WbGeometry;
class WbWrenAbstractResizeManipulator;
class WbPlane;
class WbSphere;
class WbVector3;

// WbDragResizeHandleEvent class (abstract) //
//////////////////////////////////////////////
class WbDragResizeHandleEvent : public WbDragView3DEvent {
  Q_OBJECT;

public:
  WbDragResizeHandleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                          WbGeometry *selectedGeometry);
  virtual ~WbDragResizeHandleEvent();
  void apply(const QPoint &currentMousePosition) override = 0;
  virtual void addActionInUndoStack() = 0;

signals:
  void aborted();  // triggers drag destruction in WbView3D

protected:
  QPoint mInitialMousePosition;
  WbGeometry *mSelectedGeometry;
  int mHandleNumber;
  WbWrenAbstractResizeManipulator *mManipulator;
  double mResizeRatio;
  double mTotalScaleRatio;
  double mMouseOffset;
  double mGeomCenterOffset;
  double mSizeValue;
  int mCoordinate;
  enum { X, Y, Z };

  void computeRatio(const QPoint &currentMousePosition);
  WbVector3 computeLocalMousePosition(const QPoint &currentMousePosition);
  double sizeValue() const { return mSizeValue; }
};

// WbRegularResizeEvent class (another abstract layer) : resize spheres, boxes, cylinders, capsules and cones
class WbRegularResizeEvent : public WbDragResizeHandleEvent {
public:
  WbRegularResizeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                       WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override = 0;
};

// Resize Sphere
class WbResizeSphereEvent : public WbRegularResizeEvent {
public:
  WbResizeSphereEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                      WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  WbSphere *mSphere;
};

// Resize Cylinder
class WbResizeCylinderEvent : public WbRegularResizeEvent {
public:
  WbResizeCylinderEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                        WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  WbCylinder *mCylinder;
};

// Resize Capsule
class WbResizeCapsuleEvent : public WbRegularResizeEvent {
public:
  WbResizeCapsuleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                       WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  WbCapsule *mCapsule;
};

// Resize Box
class WbResizeBoxEvent : public WbRegularResizeEvent {
public:
  WbResizeBoxEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber, WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  WbBox *mBox;
};

// Resize Plane
class WbResizePlaneEvent : public WbDragResizeHandleEvent {
public:
  WbResizePlaneEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                     WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  WbPlane *mPlane;
};

// Resize Cone
class WbResizeConeEvent : public WbRegularResizeEvent {
public:
  WbResizeConeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber, WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  WbCone *mCone;
};

// Resize ElevationGrid
class WbResizeElevationGridEvent : public WbRegularResizeEvent {
public:
  WbResizeElevationGridEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                             WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  WbElevationGrid *mElevationGrid;
};

// Resize IndexedFaceSet
class WbResizeIndexedFaceSetEvent : public WbRegularResizeEvent {
public:
  WbResizeIndexedFaceSetEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                              WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

protected:
  WbIndexedFaceSet *mIndexedFaceSet;
};

#endif
