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

#ifndef WB_QT_OBJECT_SPY_HPP
#define WB_QT_OBJECT_SPY_HPP

#include <QtCore/QList>
#include <QtCore/QObject>

//
// Description: Helper class used to log the signal activity of a qobject
// Notes:
//  - This class is not compiled (to use it, add it to the Makefile)
//  - This class has a dependency on the QTest module (add it in the file sytem, and in the Makefile)
//

class QSignalSpy;

class WbQtObjectSpy : public QObject {
  Q_OBJECT

public:
  explicit WbQtObjectSpy(QObject *object);
  virtual ~WbQtObjectSpy();

  void beginSignalSpy();
  void endSignalSpy();

private:
  void clearSpies();

  QObject *mObject;
  const QMetaObject *mMetaObject;
  QList<QSignalSpy *> mSignalSpies;
};

#endif
