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

#ifndef WB_MOUSE_HPP
#define WB_MOUSE_HPP

#include <QtCore/QList>
#include <QtCore/QObject>

class WbSensor;

class WbMouse : public QObject {
  Q_OBJECT

public:
  static WbMouse *create();
  static void destroy(WbMouse *mouse);  // Note: it calls the mouse destructor
  static QList<WbMouse *> mouses() { return mMouses; }

  bool left() const { return mLeft; }
  bool middle() const { return mMiddle; }
  bool right() const { return mRight; }

  double u() const { return mU; }
  double v() const { return mV; }

  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }

  void reset();
  void setRefreshRate(int rate);
  int refreshRate() const;
  bool hasPendingValue();
  bool refreshSensorIfNeeded();
  bool needToRefresh() const;

  bool hasMoved() const { return mHasMoved; }
  void setHasMoved(bool moved) { mHasMoved = moved; }
  bool hasClicked() const { return mHasClicked; }
  void setHasClicked(bool clicked) { mHasClicked = clicked; }
  bool isTracked() const { return mIsTracked; }
  void setTracked(bool tracked) { mIsTracked = tracked; }
  bool is3dPositionEnabled() const { return mIs3dPositionEnabled; }
  void set3dPositionEnabled(bool enable) { mIs3dPositionEnabled = enable; }

  void setPosition(double x, double y, double z) {
    mX = x;
    mY = y;
    mZ = z;
  }

  void setScreenPosition(double u, double v) {
    mU = u;
    mV = v;
  }

  void setLeft(bool left) { mLeft = left; }
  void setMiddle(bool middle) { mMiddle = middle; }
  void setRight(bool right) { mRight = right; }

signals:
  void changed();

private:
  static QList<WbMouse *> mMouses;

  WbMouse();
  virtual ~WbMouse();

  WbSensor *mSensor;

  bool mHasMoved;
  bool mHasClicked;
  bool mIsTracked;
  bool mIs3dPositionEnabled;

  // mouse button states
  bool mLeft;
  bool mMiddle;
  bool mRight;
  // mouse 2D position in the 3D window
  double mU;
  double mV;
  // mouse 3D position
  double mX;
  double mY;
  double mZ;
};

#endif  // WB_MOUSE_HPP
