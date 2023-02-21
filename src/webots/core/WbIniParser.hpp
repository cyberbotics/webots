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

#ifndef WB_INI_PARSER_HPP
#define WB_INI_PARSER_HPP

#include <QtCore/QStringList>

class QProcessEnvironment;

class WbIniParser {
public:
  explicit WbIniParser(const QString &filename);

  bool isValid() const { return mValid; }
  int size() const { return mKeys.size(); }
  const QString &sectionAt(int index) const { return mSections.at(index); }
  const QString &keyAt(int index) const { return mKeys.at(index); }
  const QString &valueAt(int index) const { return mValues.at(index); }
  void setValue(int index, const QString &newValue);

  // return value with the environment variables replaced by their value
  // for example: valueAt(i) -> $(WEBOTS_HOME)/lib
  //              resolvedValueAt(i) -> /usr/local/webots/lib
  QString resolvedValueAt(int index, const QProcessEnvironment &environment) const;

private:
  QStringList mSections;
  QStringList mKeys;
  QStringList mValues;
  bool mValid;
};
#endif
