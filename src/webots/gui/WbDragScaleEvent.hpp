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

class WbTransform;
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

#endif
