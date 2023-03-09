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

#ifndef WB_DRAG_VIEWPOINT_EVENT_HPP
#define WB_DRAG_VIEWPOINT_EVENT_HPP

//
// Description: classes allowing to store data related with viewpoint dragging
//

#include "WbAbstractDragEvent.hpp"
#include "WbVector3.hpp"

#include <QtCore/QPoint>

class WbViewpoint;

// WbDragViewpointEvent class (abstract) : change camera's position or orientation
///////////////////////////////////////////////////////////////////////////////////
class WbDragViewpointEvent : public WbDragKinematicsEvent {
public:
  virtual ~WbDragViewpointEvent() {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit WbDragViewpointEvent(WbViewpoint *viewpoint);
};

// Implemented classes:

// WbTranslateViewpointEvent class
class WbTranslateViewpointEvent : public WbDragViewpointEvent {
public:
  WbTranslateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, double scale);
  virtual ~WbTranslateViewpointEvent();
  void apply(const QPoint &currentMousePosition) override;

private:
  const QPoint mInitialMousePosition;
  QPoint mDifference;
  const WbVector3 mInitialCameraPosition;
  const double mScaleFactor;
};

// WbRotateViewpointEvent class
class WbRotateViewpointEvent : public WbDragViewpointEvent {
public:
  WbRotateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, bool objectPicked);
  virtual ~WbRotateViewpointEvent();
  void apply(const QPoint &currentMousePosition) override;

  static void applyToViewpoint(const QPoint &delta, const WbVector3 &rotationCenter, const WbVector3 &worldUpVector,
                               bool objectPicked, WbViewpoint *viewpoint);

private:
  QPoint mPreviousMousePosition;
  QPoint mDelta;
  const WbVector3 mWorldUpVector;
  bool mIsObjectPicked;
};

// WbZoomAndRotateViewpointEvent class
class WbZoomAndRotateViewpointEvent : public WbDragViewpointEvent {
public:
  WbZoomAndRotateViewpointEvent(const QPoint &initialMousePosition, WbViewpoint *viewpoint, const double scale);
  virtual ~WbZoomAndRotateViewpointEvent();
  void apply(const QPoint &currentMousePosition) override;

  static void applyToViewpoint(double tiltAngle, double zoom, double scaleFactor, WbViewpoint *viewpoint);

private:
  QPoint mPreviousMousePosition;
  QPoint mDelta;
  const double mZscaleFactor;
};

#endif
