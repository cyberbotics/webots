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

#include "AutomatonScene.hpp"

#include "Automaton.hpp"
#include "AutomatonObject.hpp"
#include "AutomatonWidget.hpp"
#include "Model.hpp"
#include "State.hpp"
#include "StateRepresentation.hpp"
#include "Transition.hpp"
#include "TransitionRepresentation.hpp"

#include <QtWidgets/QGraphicsSceneMouseEvent>

AutomatonScene::AutomatonScene(AutomatonWidget *automatonWidget) :
  QGraphicsScene(),
  mAutomatonWidget(automatonWidget),
  mTransitionCreationLine(NULL) {
  mModel = Model::instance();

  connect(mModel->automaton(), SIGNAL(stateCreated(State *)), this, SLOT(stateCreated(State *)));
  connect(mModel->automaton(), SIGNAL(stateDeleted(State *)), this, SLOT(stateDeleted(State *)));
  connect(mModel->automaton(), SIGNAL(transitionCreated(Transition *)), this, SLOT(transitionCreated(Transition *)));
  connect(mModel->automaton(), SIGNAL(transitionDeleted(Transition *)), this, SLOT(transitionDeleted(Transition *)));
  connect(mAutomatonWidget, SIGNAL(enabledChanged(bool)), this, SLOT(updateEnabled(bool)));
}

void AutomatonScene::stateCreated(State *s) {
  StateRepresentation *sr = new StateRepresentation(s);
  addItem(sr);
  sr->initialize();
}

void AutomatonScene::stateDeleted(State *s) {
  foreach (QGraphicsItem *item, items()) {
    StateRepresentation *sr = dynamic_cast<StateRepresentation *>(item);
    if (sr && sr->state() == s) {
      removeItem(sr);
      delete sr;
      break;
    }
  }
  update();
}

void AutomatonScene::transitionCreated(Transition *t) {
  TransitionRepresentation *tr = new TransitionRepresentation(t);
  addItem(tr);
  tr->updatePathItem();
  tr->initialize();
}

void AutomatonScene::transitionDeleted(Transition *t) {
  foreach (QGraphicsItem *item, items()) {
    TransitionRepresentation *tr = dynamic_cast<TransitionRepresentation *>(item);
    if (tr && tr->transition() == t) {
      removeItem(tr);
      delete tr;
      break;
    }
  }
  update();
}

void AutomatonScene::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton && mAutomatonWidget->mode() == AutomatonWidget::TransitionMode) {
    mTransitionCreationLine = new QGraphicsLineItem(QLineF(event->scenePos(), event->scenePos()));
    addItem(mTransitionCreationLine);
  } else if (event->button() == Qt::LeftButton && mAutomatonWidget->mode() == AutomatonWidget::StateMode)
    mModel->automaton()->createState(event->scenePos());
  else
    QGraphicsScene::mousePressEvent(event);
}

void AutomatonScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  if (mTransitionCreationLine && mAutomatonWidget->mode() == AutomatonWidget::TransitionMode) {
    QLineF newLine(mTransitionCreationLine->line().p1(), event->scenePos());
    mTransitionCreationLine->setLine(newLine);
  }
  QGraphicsScene::mouseMoveEvent(event);
}

void AutomatonScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  if (mTransitionCreationLine && mAutomatonWidget->mode() == AutomatonWidget::TransitionMode) {
    QList<QGraphicsItem *> startItems = items(mTransitionCreationLine->line().p1());
    if (startItems.count() > 0 && startItems.first() == mTransitionCreationLine)
      startItems.removeFirst();
    QList<QGraphicsItem *> endItems = items(mTransitionCreationLine->line().p2());
    if (endItems.count() > 0 && endItems.first() == mTransitionCreationLine)
      endItems.removeFirst();

    if (startItems.count() > 0 && endItems.count() > 0 && startItems.first() != endItems.first()) {
      StateRepresentation *start = dynamic_cast<StateRepresentation *>(startItems.first());
      StateRepresentation *end = dynamic_cast<StateRepresentation *>(endItems.first());
      if (start && end) {
        QPointF midPoint(mTransitionCreationLine->line().pointAt(0.5));
        mModel->automaton()->createTransition(midPoint, start->state(), end->state());
      }
    }
  }

  if (mTransitionCreationLine) {
    removeItem(mTransitionCreationLine);
    delete mTransitionCreationLine;
    mTransitionCreationLine = NULL;
  }

  QGraphicsScene::mouseReleaseEvent(event);
}

QGraphicsItem *AutomatonScene::findRepresentationFromAutomatonObject(AutomatonObject *o) const {
  foreach (QGraphicsItem *item, items()) {
    StateRepresentation *sr = dynamic_cast<StateRepresentation *>(item);
    TransitionRepresentation *tr = dynamic_cast<TransitionRepresentation *>(item);
    if (sr && sr->state() == o)
      return item;
    else if (tr && tr->transition() == o)
      return item;
  }
  return NULL;
}

StateRepresentation *AutomatonScene::findStateRepresentationFromState(State *s) const {
  return static_cast<StateRepresentation *>(findRepresentationFromAutomatonObject(s));
}

TransitionRepresentation *AutomatonScene::findTransitionRepresentationFromTransition(Transition *t) const {
  return static_cast<TransitionRepresentation *>(findRepresentationFromAutomatonObject(t));
}

void AutomatonScene::updateEnabled(bool enabled) {
  foreach (QGraphicsItem *item, items()) {
    StateRepresentation *sr = dynamic_cast<StateRepresentation *>(item);
    TransitionRepresentation *tr = dynamic_cast<TransitionRepresentation *>(item);
    if (sr || tr)
      item->setEnabled(enabled);
  }
}
