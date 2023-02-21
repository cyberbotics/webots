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
 * Description:  Class redefining a QGraphicsTextItem
 */

#ifndef AUTOMATON_OBJECT_REPRESENTATION_HPP
#define AUTOMATON_OBJECT_REPRESENTATION_HPP

#include <QtWidgets/QGraphicsTextItem>

class AutomatonObject;

class AutomatonObjectRepresentation : public QGraphicsTextItem {
  Q_OBJECT

public:
  explicit AutomatonObjectRepresentation(AutomatonObject *object);
  virtual ~AutomatonObjectRepresentation() {}

  virtual void initialize();
  QPointF computeCenter() const;
  AutomatonObject *automatonObject() const { return mAutomatonObject; }

  virtual QVariant itemChange(GraphicsItemChange change, const QVariant &value);

signals:
  void selectionChanged(bool newValue);
  void positionChanged();

public slots:
  void propagateSelection(bool newValue);
  void propagatePosition();
  void propagateName();
  void emitPositionChanged();

  void updateSelection();
  void updateName();
  void updatePosition();

private:
  virtual void focusOutEvent(QFocusEvent *event);
  virtual void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);

  AutomatonObject *mAutomatonObject;
};

#endif
