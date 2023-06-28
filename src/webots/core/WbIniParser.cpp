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

#include "WbIniParser.hpp"

#include <QtCore/QFile>
#include <QtCore/QProcessEnvironment>

WbIniParser::WbIniParser(const QString &filename) {
  QFile file(filename);
  QString line;
  QString section;
  file.open(QIODevice::ReadOnly);
  mValid = true;
  while (!file.atEnd()) {
    line = file.readLine();
    line = line.trimmed();

    if (!line.startsWith(';')) {
      if (line.startsWith('[')) {  // section declaration
        if (line.contains(';'))
          line = line.split(';')[0];
        line.remove('[');
        line.remove(']');
        line = line.trimmed();
        section = line;
      } else if (line.contains('=')) {                                      // key definition
        if (line.contains(';') && line.indexOf(';') < line.indexOf('=')) {  // illegal key definition
          mValid = false;
          break;
        }
        mSections.append(section.toLower());
        int equalPosition = line.indexOf("=");
        mKeys.append(line.mid(0, equalPosition).trimmed());
        QString value = line.mid(equalPosition + 1).trimmed();
        if (value.startsWith('"')) {  // preserves semi-colons inside double-quotes, remove double-quotes
          int i = value.lastIndexOf(';');
          int j = value.lastIndexOf('"');
          if (j != 0) {   // otherwise we are missing the final double-quote
            if (i > j) {  // the semi-colon introduces a comment, should be removed
              value = value.mid(1, i - 1).trimmed();
              value.chop(1);  // remove last double-quote
            } else            // the semi-colon is inside the value, should not be removed
              value = value.mid(1, j - 1);
          }
        } else
          value = value.mid(0, value.indexOf(';'));
        mValues.append(value.trimmed());
      }
    }
  }
  file.close();
}

void WbIniParser::setValue(int index, const QString &newValue) {
  mValues[index] = newValue;
}

QString WbIniParser::resolvedValueAt(int index, const QProcessEnvironment &environment) const {
  QString value = valueAt(index);
  if (sectionAt(index) == "environment variables with relative paths" ||
      sectionAt(index) == "environment variables with paths") {
#ifdef _WIN32
    QString newWindowsValue = value;
    newWindowsValue.replace(':', ';');
    newWindowsValue.replace('/', '\\');
    value = newWindowsValue;
#endif
  }
  const int count = value.count("$(");
  int position = 0;
  for (int j = 0; j < count; ++j) {
    int startIndex = value.indexOf("$(", position);
    // this test avoid errors caused by multiple instances of the same reference
    if (startIndex == -1)
      break;
    const int endIndex = value.indexOf(')', startIndex);
    const QString environmentKey = value.mid(startIndex + 2, endIndex - startIndex - 2);
    const QString &environmentValue = environment.value(environmentKey);
    QString newValue = value;
    newValue.replace("$(" + environmentKey + ")", environmentValue);
    if (sectionAt(index) == "environment variables with relative paths" ||
        sectionAt(index) == "environment variables with paths") {
#ifndef _WIN32
      newValue.replace("::", ":");
      if (newValue.startsWith(':'))
        newValue.remove(0, 1);
      if (newValue.endsWith(':'))
        newValue.remove(newValue.size() - 1, 1);
#else
      newValue.replace(";;", ";");
      if (newValue.startsWith(';'))
        newValue.remove(0, 1);
      if (newValue.endsWith(';'))
        newValue.remove(newValue.size() - 1, 1);
#endif
    }
    value = newValue;
    position = startIndex + environmentValue.length();
  }
  return value;
}
