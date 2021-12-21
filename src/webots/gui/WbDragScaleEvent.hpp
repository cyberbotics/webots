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

#ifndef WB_DRAG_SCALE_EVENT_HPP
#define WB_DRAG_SCALE_EVENT_HPP

//
// Description: classes allowing to store data related with rescale mouse dragging
//

#include "WbDragResizeEvent.hpp"
#include "WbVariant.hpp"
#include "WbVector2.hpp"
#include "WbVector3.hpp"

#include <QtCore/QObject>
#include <QtCore/QPoint>

class WbAbstractTransform;
class WbCone;
class WbCylinder;
class WbGeometry;
class WbScaleManipulator;
class WbViewpoint;

// Scale Cylinder
class WbRescaleCylinderEvent : public WbRegularResizeEvent {
public:
  WbRescaleCylinderEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                         WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  WbCylinder *mCylinder;
};

// Scale Capsule
class WbRescaleCapsuleEvent : public WbResizeCapsuleEvent {
public:
  WbRescaleCapsuleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                        WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Box
class WbRescaleBoxEvent : public WbResizeBoxEvent {
public:
  WbRescaleBoxEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber, WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Plane
class WbRescalePlaneEvent : public WbResizePlaneEvent {
public:
  WbRescalePlaneEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                      WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale Cone
class WbRescaleConeEvent : public WbRegularResizeEvent {
public:
  WbRescaleConeEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                     WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;

private:
  WbCone *mCone;
};

// Scale ElevationGrid
class WbRescaleElevationGridEvent : public WbResizeElevationGridEvent {
public:
  WbRescaleElevationGridEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                              WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Scale IndexedFaceSet
class WbRescaleIndexedFaceSetEvent : public WbResizeIndexedFaceSetEvent {
public:
  WbRescaleIndexedFaceSetEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                               WbGeometry *selectedGeometry);
  void apply(const QPoint &currentMousePosition) override;
  void addActionInUndoStack() override;
};

// Drag changing the scale field of a WbTransform
class WbDragScaleHandleEvent : public WbDragView3DEvent {
  Q_OBJECT;

public:
  WbDragScaleHandleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                         WbAbstractTransform *selectedTransform);
  virtual ~WbDragScaleHandleEvent();
  void apply(const QPoint &currentMousePosition) override;
  virtual void addActionInUndoStack();

signals:
  void aborted();  // triggers drag destruction in WbView3D

protected:
  WbAbstractTransform *mTransform;
  int mHandleNumber;
  WbScaleManipulator *mManipulator;
  WbVariant mInitialScale;
  double mScaleRatio;
  double mLocalMouseOffset;
  WbVector3 mAttachedHandlePosition, mOppositeHandlePosition;
  WbVector2 mAttachedHandleProjection, mOppositeHandleProjection;
  WbVector2 mMousePositionOffset;
  int mCoordinate;
  float mTotalScale;
  enum { X, Y, Z };

  void computeRatio(const QPoint &currentMousePosition);
  void computeHandlesPositions(const QPoint &currentMousePosition, WbVector3 &attachedHandlePos, WbVector3 &oppositeHandlePos,
                               WbVector3 &localMousePos);
};

// Uniform scale
class WbUniformScaleEvent : public WbDragScaleHandleEvent {
public:
  WbUniformScaleEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, int handleNumber,
                      WbAbstractTransform *selectedTransform);
  void apply(const QPoint &currentMousePosition) override;
};

#endif
