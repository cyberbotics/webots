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

#ifndef WB_ABSTRACT_DRAG_EVENT_HPP
#define WB_ABSTRACT_DRAG_EVENT_HPP

//
// Description: abstract classes allowing to store data related with the different kind of mouse dragging
//              (so far used only for mouse dragging impacting the 3D view, i.e. WbDragViewpoint)
//

#include <QtCore/QObject>

#include <QtCore/QPoint>

class WbVector2;
class WbVector3;
class WbWrenLabelOverlay;
class WbViewpoint;

// WbDragEvent class
class WbDragEvent : public QObject {
  Q_OBJECT;

public:
  virtual ~WbDragEvent() {}
  virtual void apply(const QPoint &currentMousePosition) = 0;

  // test numerical limits supported by bounding boxes
  static const float cFloatMax;
  static bool exceedsFloatMax(const WbVector3 &v);
  static bool exceedsFloatMax(double x);
  static bool exceedsFloatMax(float x);

protected:
  WbDragEvent();
};

// Abstract class for drag events impacting the 3D-View //
//////////////////////////////////////////////////////////

// WbDragView3DEvent class
class WbDragView3DEvent : public WbDragEvent {
public:
  virtual ~WbDragView3DEvent() {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit WbDragView3DEvent(WbViewpoint *viewpoint);

  static WbVector2 clampLabelPosition(const float x, const float y, const WbWrenLabelOverlay *overlay);

  WbViewpoint *mViewpoint;
  float mViewDistanceUnscaling;
};

// Abstract class for drag events involving kinematic changes only: camera moves, non-physical solid or fluid displacement //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// WbDragKinematicsEvent class (abstract)
/////////////////////////////////////////
class WbDragKinematicsEvent : public WbDragView3DEvent {
public:
  virtual ~WbDragKinematicsEvent() {}
  void apply(const QPoint &currentMousePosition) override = 0;

protected:
  explicit WbDragKinematicsEvent(WbViewpoint *viewpoint);
};

#endif
