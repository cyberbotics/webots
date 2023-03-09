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
 * Description:  Implementation of the AutomatonWidget.hpp functions
 */

#include "AutomatonWidget.hpp"

#include "AutomatonScene.hpp"

#include <QtCore/qmath.h>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QScrollBar>

AutomatonWidget::AutomatonWidget(QWidget *parent) :
  QGraphicsView(parent),
  mLastMovePos(NULL),
  mMode(SelectionMode),
  mZoomFactor(0.0),
  mWheelAttenuation(0.1) {
  mAutomatonScene = new AutomatonScene(this);
  setRenderHint(QPainter::Antialiasing);
  setScene(mAutomatonScene);
  setSceneRect(-1000, -1000, 2000, 2000);
  setMode(SelectionMode);
  centerOn(0, 0);
  resetMatrix();
}

QSize AutomatonWidget::sizeHint() const {
  static QSize s(500, 500);
  return s;
}

AutomatonWidget::~AutomatonWidget() {
  delete mAutomatonScene;
}

void AutomatonWidget::wheelEvent(QWheelEvent *e) {
  double direction = 1.0;
  if (e->angleDelta().y() < 0)
    direction = -1.0;
  mZoomFactor += mWheelAttenuation * direction;

  resetMatrix();
}

void AutomatonWidget::mouseMoveEvent(QMouseEvent *event) {
  if (mLastMovePos) {
    QScrollBar *hBar = horizontalScrollBar();
    QScrollBar *vBar = verticalScrollBar();
    QPoint delta = event->pos() - *mLastMovePos;
    *mLastMovePos = event->pos();
    hBar->setValue(hBar->value() + (isRightToLeft() ? delta.x() : -delta.x()));
    vBar->setValue(vBar->value() - delta.y());
  } else
    QGraphicsView::mouseMoveEvent(event);
}

void AutomatonWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::RightButton) {
    mLastMovePos = new QPoint(event->pos());
    setCursor(Qt::ClosedHandCursor);
  }
  QGraphicsView::mousePressEvent(event);
}

void AutomatonWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (mLastMovePos && event->button() == Qt::RightButton) {
    delete mLastMovePos;
    mLastMovePos = NULL;
  } else
    QGraphicsView::mouseReleaseEvent(event);

  updateCursor();
}

void AutomatonWidget::changeEvent(QEvent *event) {
  if (event->type() == QEvent::EnabledChange)
    emit enabledChanged(isEnabled());
  QGraphicsView::changeEvent(event);
}

void AutomatonWidget::updateCursor() {
  if (mLastMovePos)
    setCursor(Qt::ClosedHandCursor);
  else if (mMode == SelectionMode)
    setCursor(Qt::ArrowCursor);
  else
    setCursor(Qt::CrossCursor);
}

void AutomatonWidget::resetMatrix() {
  double scale = qPow(2.0, mZoomFactor);
  QTransform transform;
  transform.scale(scale, scale);
  setTransform(transform);
}

void AutomatonWidget::setMode(int mode) {
  mMode = mode;

  if (mMode == SelectionMode)
    setDragMode(RubberBandDrag);
  else
    setDragMode(NoDrag);

  updateCursor();
}
