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
 * Description:  Implementation of the Automaton.hpp functions
 */

#include "Automaton.hpp"
#include "AutomatonObject.hpp"
#include "RobotActuatorCommand.hpp"
#include "RobotSensorCondition.hpp"
#include "State.hpp"
#include "Transition.hpp"

#include <QtCore/QRegularExpression>

Automaton::Automaton() : QObject(), mCurrentState(NULL) {
}

Automaton::~Automaton() {
  deleteAllObjects();
}

State *Automaton::createState(const QPointF &pos) {
  State *s = new State(pos);
  addState(s);
  return s;
}

void Automaton::addState(State *s) {
  mObjects.append(s);
  emit stateCreated(s);
  connect(s, SIGNAL(selectionChanged()), this, SLOT(emitSelectionChanged()));
}

Transition *Automaton::createTransition(const QPointF &pivotPos, State *startState, State *endState) {
  Transition *t = new Transition(pivotPos, startState, endState);
  addTransition(t);
  return t;
}

void Automaton::addTransition(Transition *t) {
  t->startState()->addTransition(t);
  t->endState()->addTransition(t);
  mObjects.append(t);
  emit transitionCreated(t);
  connect(t, SIGNAL(selectionChanged()), this, SLOT(emitSelectionChanged()));
}

void Automaton::deleteAllObjects() {
  QList<State *> states;
  foreach (AutomatonObject *object, mObjects) {
    State *state = dynamic_cast<State *>(object);
    if (state) {
      states.append(state);

      if (state->isInitial()) {
        // reset sensors use
        QList<Transition *> transitions = state->transitions();
        if (!transitions.isEmpty() && transitions.at(0)->sensorCondition())
          transitions.at(0)->sensorCondition()->resetSensorsUseNotification();
      }
    }
  }
  foreach (State *state, states)
    deleteState(state);
  emitSelectionChanged();
  AutomatonObject::resetCurrentGlobalId();
}

void Automaton::deleteState(State *s) {
  foreach (Transition *t, s->transitions())
    deleteTransition(t);

  foreach (AutomatonObject *object, mObjects) {
    if (object == s) {
      emit stateDeleted(s);
      mObjects.removeOne(s);
      delete s;
      emitSelectionChanged();
      return;
    }
  }
}

void Automaton::deleteTransition(Transition *t) {
  t->startState()->removeTransition(t);
  t->endState()->removeTransition(t);
  foreach (AutomatonObject *object, mObjects) {
    if (object == t) {
      emit transitionDeleted(t);
      mObjects.removeOne(t);
      delete t;
      emitSelectionChanged();
      return;
    }
  }
}

void Automaton::setSelectedStateAsInitial() {
  foreach (AutomatonObject *object, mObjects) {
    State *state = dynamic_cast<State *>(object);
    if (state)
      state->setInitial(state->isSelected());
  }
}

void Automaton::deleteSelectedObjects() {
  // first pass: delete selected transitions
  QList<Transition *> transitions;
  foreach (AutomatonObject *object, mObjects) {
    if (object->isSelected()) {
      Transition *transition = dynamic_cast<Transition *>(object);
      if (transition)
        transitions.append(transition);
    }
  }
  foreach (Transition *transition, transitions)
    deleteTransition(transition);

  // second pass: delete selected states
  QList<State *> states;
  foreach (AutomatonObject *object, mObjects) {
    if (object->isSelected()) {
      State *state = dynamic_cast<State *>(object);
      if (state)
        states.append(state);
    }
  }
  foreach (State *state, states)
    deleteState(state);
}

int Automaton::computeNumberOfSelectedStates() const {
  int counter = 0;
  foreach (AutomatonObject *object, mObjects)
    if (object->isSelected()) {
      State *state = dynamic_cast<State *>(object);
      if (state)
        counter++;
    }
  return counter;
}

int Automaton::computeNumberOfSelectedItems() const {
  int counter = 0;
  foreach (AutomatonObject *object, mObjects)
    if (object->isSelected())
      counter++;
  return counter;
}

State *Automaton::findStateById(int id) const {
  foreach (AutomatonObject *object, mObjects)
    if (object->uniqueId() == id) {
      State *state = dynamic_cast<State *>(object);
      if (state)
        return state;
    }
  return NULL;
}

AutomatonObject *Automaton::findSelectedObject() const {
  int counter = 0;
  AutomatonObject *selectedObject = NULL;
  foreach (AutomatonObject *object, mObjects) {
    if (object->isSelected()) {
      selectedObject = object;
      counter++;
    }
  }

  if (counter == 1)
    return selectedObject;
  else
    return NULL;
}

void Automaton::emitSelectionChanged() {
  emit selectionChanged();
}

bool Automaton::prepareToRun() {
  int counter = 0;
  foreach (AutomatonObject *object, mObjects) {
    State *state = dynamic_cast<State *>(object);
    if (state) {
      if (state->isInitial()) {
        counter++;
        mCurrentState = state;
      }
      state->setCurrent(state->isInitial());
    }
  }

  if (counter == 1) {
    mCurrentState->actuatorCommand()->sendCommands();
    foreach (const Transition *nextTransition, mCurrentState->transitions())
      nextTransition->sensorCondition()->reset();
    return true;
  } else {
    mCurrentState = NULL;
    return false;
  }
}

void Automaton::run() {
  if (!mCurrentState)
    return;

  foreach (const Transition *transition, mCurrentState->transitions()) {
    if (transition->startState() == mCurrentState && transition->sensorCondition()->isFired()) {
      mCurrentState->setCurrent(false);
      mCurrentState = transition->endState();
      mCurrentState->setCurrent(true);
      mCurrentState->actuatorCommand()->sendCommands();
      foreach (const Transition *nextTransition, mCurrentState->transitions())
        nextTransition->sensorCondition()->reset();
      return;
    }
  }
}

void Automaton::stop() {
  mCurrentState = NULL;
}

void Automaton::fromString(const QString &string) {
  QStringList lines = string.split(QRegularExpression("\n"), Qt::SkipEmptyParts);

  foreach (const QString &line, lines) {
    if (line.startsWith('S')) {
      State *s = new State(QPointF());
      try {
        s->fromString(line);
      } catch (const QString &error) {
        delete s;
        throw;
      }
      addState(s);
    } else if (line.startsWith('T')) {
      Transition *t = new Transition(QPointF(), NULL, NULL);
      try {
        t->fromString(line);
      } catch (const QString &error) {
        delete t;
        throw;
      }
      addTransition(t);
    }
  }
}

QString Automaton::toString() const {
  QString out;
  foreach (AutomatonObject *object, mObjects) {
    State *state = dynamic_cast<State *>(object);
    if (state)
      out += state->toString() + "\n";
  }
  foreach (AutomatonObject *object, mObjects) {
    Transition *transition = dynamic_cast<Transition *>(object);
    if (transition)
      out += transition->toString() + "\n";
  }

  return out;
}

// backward compatibility code
void Automaton::fromStringVersion3(const QString &string) {
  QStringList lines = string.split(QRegularExpression("\n"));

  lines.removeFirst();  // remove header

  bool readState = true;  // read states until an empty line, then read transitions

  while (!lines.isEmpty()) {
    const QString &line = lines.first();

    if (line.isEmpty()) {
      readState = false;
    } else if (readState) {
      State *s = new State(QPointF());
      try {
        s->fromStringVersion3(line);
      } catch (const QString &error) {
        delete s;
        throw;
      }
      addState(s);
    } else {  // readTransition
      Transition *t = new Transition(QPointF(), NULL, NULL);
      try {
        t->fromStringVersion3(line);
      } catch (const QString &error) {
        delete t;
        throw;
      }
      addTransition(t);
    }
    lines.removeFirst();
  }
}
