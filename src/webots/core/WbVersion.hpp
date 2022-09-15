// Copyright 1996-2022 Cyberbotics Ltd.
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

#ifndef WB_VERSION_HPP
#define WB_VERSION_HPP

//
// Description: store a generic version structure major.minor.revision
//              and support Webots specific version R2018a revision 1
//

#include <QtCore/QString>

class WbVersion {
public:
  explicit WbVersion(int major = 0, int minor = 0, int revision = 0, bool webots = false);
  WbVersion(const WbVersion &other);
  virtual ~WbVersion() {}

  void setMajor(int major) { mMajor = major; }
  void setMinor(int minor) { mMinor = minor; }
  void setRevision(int revision) { mRevision = revision; }
  // extract the version from a string using the given prefix and suffix to build the regular expression
  // e.g. pattern "#VRML_SIM V8.6 utf8" and prefix "^VRML(_...|) " and suffix "( utf8|)$""
  bool fromString(const QString &text, const QString &prefix = "", const QString &suffix = "");

  int majorNumber() const { return mMajor; }
  int minorNumber() const { return mMinor; }
  int revisionNumber() const { return mRevision; }
  const QString &commit() const { return mCommit; }

  // Write version in a verbose way
  // if revision is false, only major and minor information are included.
  // In general the string will have the form "8.6.3".
  // Webots version is a special case and the string will have the form
  // "R2018a revision 1" or "R2018a" if revision is false, and
  // "2018.0.1 if digitsOnly is true ('a' = 0)
  QString toString(bool revision = true, bool digitsOnly = false, bool nightly = false) const;

  // copy
  WbVersion &operator=(const WbVersion &other) {
    mMajor = other.mMajor;
    mMinor = other.mMinor;
    mRevision = other.mRevision;
    mIsWebots = other.mIsWebots;
    mCommit = other.mCommit;
    mDate = other.mDate;
    return *this;
  }
  // comparison operators
  friend bool operator==(const WbVersion &l, const WbVersion &r) {
    return l.mMajor == r.mMajor && l.mMinor == r.mMinor && l.mRevision == r.mRevision;
  }
  friend bool operator!=(const WbVersion &l, const WbVersion &r) { return !(l == r); }
  friend bool operator<(const WbVersion &l, const WbVersion &r) {
    // TODO: we can certainly find a cleaner algorithm
    if (l.mMajor < r.mMajor)
      return true;
    else if (l.mMajor > r.mMajor)
      return false;
    else {  // l.mMajor == r.mMajor
      if (l.mMinor < r.mMinor)
        return true;
      else if (l.mMinor > r.mMinor)
        return false;
      else  // l.mMinor == r.mMinor
        return l.mRevision < r.mRevision;
    }
  }
  friend bool operator>(const WbVersion &l, const WbVersion &r) { return r < l; }
  friend bool operator<=(const WbVersion &l, const WbVersion &r) { return !(r < l); }
  friend bool operator>=(const WbVersion &l, const WbVersion &r) { return !(l < r); }

private:
  int mMajor;
  int mMinor;  // in case of Webots version 0 corresponds to 'a'
  int mRevision;
  QString mCommit;
  QString mDate;

  bool mIsWebots;
};

#endif
