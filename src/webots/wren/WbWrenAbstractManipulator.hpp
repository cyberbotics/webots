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

#ifndef WB_WREN_ABSTRACT_MANIPULATOR_HPP
#define WB_WREN_ABSTRACT_MANIPULATOR_HPP

//
// Description: abstract class implementing basic function for a manipulator
//              that acts on 3 dimension
//
#include <wren/shader_program.h>
#include <wren/transform.h>
#include <wren/viewport.h>

#include <QtCore/QObject>
#include <QtCore/QVector>

class WbVector3;

class WbWrenAbstractManipulator : public QObject {
  Q_OBJECT

public:
  virtual ~WbWrenAbstractManipulator();

  // Getters
  bool isAttached() const { return mIsVisible; }
  bool isActive() const { return mIsActive; }

  // Setters
  void attachTo(WrTransform *parent);
  void setActive(bool b) { mIsActive = b; }
  void updateHandleScale(const double *scale);

  // Visibility
  virtual void show();
  virtual void highlightAxis(int index) {}
  virtual void showNormal();

  // Others
  virtual int coordinate(int handleNumber) const = 0;
  virtual int coordinateToHandleNumber(int coord) = 0;
  virtual const WbVector3 &coordinateVector(int handleNumber) const = 0;
  virtual WbVector3 relativeHandlePosition(int handleNumber) const = 0;

  void computeHandleScaleFromViewportSize();
  static void setViewport(WrViewport *viewport);

public slots:
  virtual void hide();

protected:
  WbWrenAbstractManipulator(int numberOfHandles);

  WrTransform *mRootNode;
  WrTransform *mTransform;
  WrShaderProgram *mHandlesShader;
  WrShaderProgram *mHandlesPickingShader;

  bool mIsVisible;
  bool mOriginScaleFactorNeeded;
  float mScale;
  int mNumberOfHandles;

protected slots:
private:
  WbWrenAbstractManipulator(const WbWrenAbstractManipulator &original);
  WbWrenAbstractManipulator &operator=(const WbWrenAbstractManipulator &original);

  bool mIsActive;
  static WrViewport *cViewport;
};

#endif  // WB_WREN_ABSTRACT_MANIPULATOR_HPP
