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
 * Description:  Class defining the Automaton scene
 */

#ifndef AUTOMATON_SCENE_HPP
#define AUTOMATON_SCENE_HPP

#include <QtCore/QList>
#include <QtWidgets/QGraphicsScene>

class Model;
class State;
class StateRepresentation;
class Transition;
class TransitionRepresentation;
class AutomatonObject;
class AutomatonWidget;

class AutomatonScene : public QGraphicsScene {
  Q_OBJECT

public:
  explicit AutomatonScene(AutomatonWidget *automatonWidget);
  virtual ~AutomatonScene() {}
  StateRepresentation *findStateRepresentationFromState(State *s) const;
  TransitionRepresentation *findTransitionRepresentationFromTransition(Transition *t) const;

public slots:
  void stateCreated(State *s);
  void stateDeleted(State *s);
  void transitionCreated(Transition *t);
  void transitionDeleted(Transition *t);
  void updateEnabled(bool enabled);

private:
  virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
  virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
  virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

  QGraphicsItem *findRepresentationFromAutomatonObject(AutomatonObject *o) const;

  Model *mModel;
  AutomatonWidget *mAutomatonWidget;
  QGraphicsLineItem *mTransitionCreationLine;
};

#endif
