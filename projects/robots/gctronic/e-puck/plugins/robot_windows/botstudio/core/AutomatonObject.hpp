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
 * Description:  Class defining the model of an automaton object
 */

#ifndef AUTOMATON_OBJECT_HPP
#define AUTOMATON_OBJECT_HPP

#include <QtCore/QObject>
#include <QtCore/QPointF>

class AutomatonObject : public QObject {
  Q_OBJECT

public:
  explicit AutomatonObject(const QPointF &position);
  virtual ~AutomatonObject() {}

  int uniqueId() const { return mUniqueId; }
  const QString &name() const { return mName; }
  const QPointF &position() const { return mPosition; }
  bool isSelected() const { return mIsSelected; }
  static void resetCurrentGlobalId() { cCurrentGlobalId = 0; }

  void setName(const QString &name);
  void setSelected(bool isSelected);
  void setPosition(const QPointF &position);

signals:
  void nameUpdated();
  void selectionChanged();
  void positionUpdated();

protected:
  static void computeCurrentGlobalId(int id);
  void setUniqueId(int id) { mUniqueId = id; }

private:
  static int cCurrentGlobalId;

  QPointF mPosition;
  QString mName;
  int mUniqueId;
  bool mIsSelected;
};

#endif
