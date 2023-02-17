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

#include "WbQtObjectSpy.hpp"

#include <QtCore/QDebug>
#include <QtCore/QMetaMethod>
#include <QtTest/QSignalSpy>

WbQtObjectSpy::WbQtObjectSpy(QObject *object) : mObject(object) {
  mMetaObject = mObject->metaObject();
}

WbQtObjectSpy::~WbQtObjectSpy() {
  clearSpies();
}

void WbQtObjectSpy::clearSpies() {
  qDeleteAll(mSignalSpies);
  mSignalSpies.clear();
}

void WbQtObjectSpy::beginSignalSpy() {
  for (int i = 0; i < mMetaObject->methodCount(); i++) {
    const QMetaMethod &method = mMetaObject->method(i);
    if (method.methodType() == QMetaMethod::Signal) {
      QString signalSignature = QString("2%1").arg(method.signature());
      mSignalSpies << new QSignalSpy(mObject, signalSignature.toLatin1().data());
    }
  }
}

void WbQtObjectSpy::endSignalSpy() {
  foreach (QSignalSpy *spy, mSignalSpies) {
    QString debugMessage;
    debugMessage += mMetaObject->className();
    debugMessage += QString("[%1]").arg((long)mObject);
    debugMessage += "::";
    debugMessage += spy->signal();
    debugMessage += " was emitted";
    qDebug() << debugMessage;
  }
  clearSpies();
}
