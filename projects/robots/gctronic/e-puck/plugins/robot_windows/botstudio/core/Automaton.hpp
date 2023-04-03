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
 * Description:  Class defining the model of the automaton
 */

#ifndef AUTOMATON_HPP
#define AUTOMATON_HPP

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QStringList>

class State;
class Transition;
class AutomatonObject;

class Automaton : public QObject {
  Q_OBJECT

public:
  Automaton();
  virtual ~Automaton();

  State *createState(const QPointF &pos);
  void deleteState(State *s);

  Transition *createTransition(const QPointF &pivotPos, State *startState, State *endState);
  void deleteTransition(Transition *t);

  void setSelectedStateAsInitial();
  void deleteSelectedObjects();
  void deleteAllObjects();

  int computeNumberOfSelectedStates() const;
  int computeNumberOfSelectedItems() const;
  AutomatonObject *findSelectedObject() const;
  State *findStateById(int id) const;

  bool prepareToRun();
  void run();
  void stop();

  void fromString(const QString &string);
  void fromStringVersion3(const QString &string);  // backward compatibility code
  QString toString() const;

signals:
  void stateCreated(State *s);
  void stateDeleted(State *s);
  void transitionCreated(Transition *t);
  void transitionDeleted(Transition *t);
  void selectionChanged();

private slots:
  void emitSelectionChanged();

private:
  void addState(State *s);
  void addTransition(Transition *t);

  QList<AutomatonObject *> mObjects;
  State *mCurrentState;
};

#endif
