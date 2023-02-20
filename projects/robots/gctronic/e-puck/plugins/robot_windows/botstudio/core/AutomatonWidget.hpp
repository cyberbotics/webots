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

/*
 * Description:  Class defining the view/controller of the Automaton model
 */

#ifndef AUTOMATON_WIDGET_HPP
#define AUTOMATON_WIDGET_HPP

#include <QtWidgets/QGraphicsView>

class Model;
class State;
class AutomatonScene;

QT_BEGIN_NAMESPACE
class QPoint;
QT_END_NAMESPACE

class AutomatonWidget : public QGraphicsView {
  Q_OBJECT

public:
  enum { SelectionMode, StateMode, TransitionMode };

  explicit AutomatonWidget(QWidget *parent = NULL);
  virtual ~AutomatonWidget();

  void setMode(int mode);
  int mode() const { return mMode; }

  virtual QSize sizeHint() const;

signals:
  void enabledChanged(bool enabled);

private:
  virtual void wheelEvent(QWheelEvent *e);
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void changeEvent(QEvent *event);

  void resetMatrix();
  void updateCursor();

  QPoint *mLastMovePos;
  int mMode;
  AutomatonScene *mAutomatonScene;
  double mZoomFactor;
  double mWheelAttenuation;
};

#endif
