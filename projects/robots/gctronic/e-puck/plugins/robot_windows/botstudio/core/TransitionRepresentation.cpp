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
 * Description:  Implementation of the TransitionRepresentation.hpp functions
 */

#include "TransitionRepresentation.hpp"

#include "AutomatonScene.hpp"
#include "CommonProperties.hpp"
#include "State.hpp"
#include "StateRepresentation.hpp"
#include "Transition.hpp"

#include <QtCore/qmath.h>
#include <QtGui/QPainter>
#include <QtGui/QTextDocument>
#include <QtWidgets/QStyle>
#include <QtWidgets/QWidget>

static const int ROUNDED_RECT_RADIUS = 10;
static const double ARROW_LENGTH = 15.0;
static const double ARROW_ANGLE = 0.5;
static const double ARROW_PERCENTAGE = 1.0 / 4.0;
static const double PIVOT_PERCENTAGE = 1.5;

TransitionRepresentation::TransitionRepresentation(Transition *t) :
  AutomatonObjectRepresentation(t),
  mPathItem(NULL),
  mAutomatonScene(NULL),
  mStartState(NULL),
  mEndState(NULL),
  mIsInitialized(false) {
  setZValue(2.0);

  QFont f = font();
  f.setPointSize(8);
  setFont(f);
}

TransitionRepresentation::~TransitionRepresentation() {
  if (mPathItem)
    delete mPathItem;
}

// init should be called after the attachment of the object over the scene
void TransitionRepresentation::initialize() {
  if (mIsInitialized)
    return;

  AutomatonObjectRepresentation::initialize();

  mAutomatonScene = static_cast<AutomatonScene *>(scene());
  mStartState = mAutomatonScene->findStateRepresentationFromState(transition()->startState());
  mEndState = mAutomatonScene->findStateRepresentationFromState(transition()->endState());
  if (!mStartState || !mEndState)
    qFatal("Error on transition \"%s\": its start and/or end states are invalid",
           transition()->name().toLocal8Bit().constData());
  mPathItem = new QGraphicsPathItem;
  mAutomatonScene->addItem(mPathItem);

  connect(this, SIGNAL(positionChanged()), this, SLOT(updatePathItem()));
  connect(document(), SIGNAL(contentsChanged()), this, SLOT(updatePathItem()));
  connect(mStartState, SIGNAL(positionChanged()), this, SLOT(updatePathItem()));
  connect(mEndState, SIGNAL(positionChanged()), this, SLOT(updatePathItem()));
  connect(mStartState->document(), SIGNAL(contentsChanged()), this, SLOT(updatePathItem()));
  connect(mEndState->document(), SIGNAL(contentsChanged()), this, SLOT(updatePathItem()));

  mIsInitialized = true;
}

void TransitionRepresentation::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
  painter->setPen(Qt::black);
  QPainterPath path;

  QColor color = CommonProperties::transitionColor();
  if (isSelected())
    color = CommonProperties::selectionColor();
  if (!isEnabled())
    color = widget->style()->standardPalette().color(QPalette::Window);

  path.addRoundedRect(boundingRect(), ROUNDED_RECT_RADIUS, ROUNDED_RECT_RADIUS);
  painter->fillPath(path, QBrush(color));
  painter->drawPath(path);

  AutomatonObjectRepresentation::paint(painter, option, widget);
}

Transition *TransitionRepresentation::transition() const {
  return static_cast<Transition *>(automatonObject());
}

void TransitionRepresentation::updatePathItem() {
  if (!mIsInitialized)
    initialize();

  // set the arrow bezier curve
  QPainterPath path;
  QPointF start(mStartState->computeCenter());
  QPointF end(mEndState->computeCenter());
  QPointF pivot(computeCenter());
  QPointF statesCenter((start + end) / 2.0);
  QLineF statesCenterToPivot(statesCenter, pivot);
  QPointF extendedPivot(statesCenterToPivot.pointAt(PIVOT_PERCENTAGE));
  path.moveTo(start);
  path.cubicTo(extendedPivot, extendedPivot, end);

  // compute the point where the arrow is (at a specific percentage of the bezier curve)
  QPointF arrowCenter(path.pointAtPercent(ARROW_PERCENTAGE));
  double angleAtArrowCenter = path.angleAtPercent(ARROW_PERCENTAGE) / 180.0 * M_PI - M_PI_2;
  QPointF arrowExtremity1(qSin(angleAtArrowCenter + ARROW_ANGLE), qCos(angleAtArrowCenter + ARROW_ANGLE));
  QPointF arrowExtremity2(qSin(angleAtArrowCenter - ARROW_ANGLE), qCos(angleAtArrowCenter - ARROW_ANGLE));

  // set the arrow lines
  path.moveTo(arrowCenter);
  path.lineTo(arrowCenter + arrowExtremity1 * ARROW_LENGTH);
  path.moveTo(arrowCenter);
  path.lineTo(arrowCenter + arrowExtremity2 * ARROW_LENGTH);

  // set the path to the QGraphicsPathItem
  mPathItem->setPath(path);
}
