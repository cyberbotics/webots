// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbVersion.hpp"

#include <QtCore/QObject>
#include <QtCore/QRegExp>

WbVersion::WbVersion(int major, int minor, int revision, bool webots) :
  mMajor(major),
  mMinor(minor),
  mRevision(revision),
  mCommit(""),
  mDate(""),
  mIsWebots(webots) {
}

WbVersion::WbVersion(const WbVersion &other) :
  mMajor(other.mMajor),
  mMinor(other.mMinor),
  mRevision(other.mRevision),
  mCommit(other.mCommit),
  mDate(other.mDate),
  mIsWebots(other.mIsWebots) {
}

bool WbVersion::fromString(const QString &text, const QString &prefix, const QString &suffix, int expressionCountInPrefix) {
  mMajor = 0;
  mMinor = 0;
  mRevision = 0;
  mCommit = "";
  mDate = "";
  mIsWebots = false;

  // Check for version format R2018 or R2018a revision 1 or R2018a-rev1 (needed in WbWebotsUpdateManager)
  QRegExp rx(prefix + "R(\\d+)([a-z])(?:\\srevision\\s(\\d+)|-rev(\\d+))?(?:-(\\w*))?(?:-(\\w*\\/\\w*\\/\\w*))?" + suffix);
  int pos = rx.indexIn(text);
  if (pos != -1) {
    mMajor = rx.cap(expressionCountInPrefix + 1).toInt();
    if (!rx.cap(expressionCountInPrefix + 2).isEmpty())
      mMinor = rx.cap(expressionCountInPrefix + 2).at(0).unicode() - QChar('a').unicode();
    if (!rx.cap(expressionCountInPrefix + 3).isEmpty())
      mRevision = rx.cap(expressionCountInPrefix + 3).toInt();
    else if (!rx.cap(expressionCountInPrefix + 4).isEmpty())
      mRevision = rx.cap(expressionCountInPrefix + 4).toInt();
    if (!rx.cap(expressionCountInPrefix + 5).isEmpty())
      mCommit = rx.cap(expressionCountInPrefix + 5);
    if (!rx.cap(expressionCountInPrefix + 6).isEmpty())
      mDate = rx.cap(expressionCountInPrefix + 6);
    mIsWebots = true;
    return true;
  }

  // Check for old version formats 7.4.3 or 8.6
  rx = QRegExp(prefix + "(\\d+).(\\d+).?(\\d+)?" + suffix);
  pos = rx.indexIn(text);
  if (pos != -1) {
    mMajor = rx.cap(expressionCountInPrefix + 1).toInt();
    mMinor = rx.cap(expressionCountInPrefix + 2).toInt();
    if (rx.captureCount() >= (expressionCountInPrefix + 3))
      mRevision = rx.cap(expressionCountInPrefix + 3).toInt();
    return true;
  }

  return false;
}

QString WbVersion::toString(bool revision, bool digitsOnly, bool nightly) const {
  if (!digitsOnly && mIsWebots) {
    QString nightlyString = (!mCommit.isEmpty() && !mDate.isEmpty() && nightly) ?
                              QString(QObject::tr(" Nightly Build %1 %2").arg(mDate).arg(mCommit)) :
                              "";
    if (revision && mRevision > 0)
      return QString("R%1%2 revision %3%4").arg(mMajor).arg(QChar(mMinor + 'a')).arg(mRevision).arg(nightlyString);
    else
      return QString("R%1%2%3").arg(mMajor).arg(QChar(mMinor + 'a')).arg(nightlyString);
  }

  if (revision)
    return QString("%1.%2.%3").arg(mMajor).arg(mMinor).arg(mRevision);
  return QString("%1.%2").arg(mMajor).arg(mMinor);
}
