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
 * Description:  Class defining the graphical representation of a state
 */

#ifndef STATE_REPRESENTATION_HPP
#define STATE_REPRESENTATION_HPP

#include "AutomatonObjectRepresentation.hpp"

class State;

class StateRepresentation : public AutomatonObjectRepresentation {
  Q_OBJECT

public:
  explicit StateRepresentation(State *s);
  virtual ~StateRepresentation() {}
  virtual void initialize();
  virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  State *state() const;

private slots:
  void callUpdate();
};

#endif
