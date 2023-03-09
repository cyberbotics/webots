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

#include "WbWindowsRegistry.hpp"

#include <cassert>

// Implementation inspired from:
// https://msdn.microsoft.com/en-us/library/windows/desktop/ms724256(v=vs.85).aspx

#define MAX_KEY_LENGTH 255
#define MAX_VALUE_NAME 16383

WbWindowsRegistry::WbWindowsRegistry(const QString &key) {
  HKEY baseKey = 0;
  QStringList keys = key.split("\\", Qt::SkipEmptyParts);
  assert(keys.length() > 0);
  QString baseString = keys.takeFirst();
  if (baseString.toUpper() == "HKEY_CLASSES_ROOT")
    baseKey = HKEY_CLASSES_ROOT;
  else if (baseString.toUpper() == "HKEY_CURRENT_USER")
    baseKey = HKEY_CURRENT_USER;
  else if (baseString.toUpper() == "HKEY_LOCAL_MACHINE")
    baseKey = HKEY_LOCAL_MACHINE;
  else if (baseString.toUpper() == "HKEY_USERS")
    baseKey = HKEY_USERS;
  else if (baseString.toUpper() == "HKEY_CURRENT_CONFIG")
    baseKey = HKEY_CURRENT_CONFIG;
  else
    assert(0);  // unsupported base key
#ifndef NDEBUG
  LONG successCode =
#endif
    RegOpenKeyEx(baseKey, keys.join("\\").toStdString().c_str(), 0, KEY_READ, &mCurrentKey);
  assert(successCode == ERROR_SUCCESS);
}

WbWindowsRegistry::~WbWindowsRegistry() {
  RegCloseKey(mCurrentKey);
}

QString WbWindowsRegistry::stringValue(const QString &name) const {
  WCHAR szBuffer[512];
  DWORD dwBufferSize = sizeof(szBuffer);
  wchar_t buffer[name.length() + 1];
  name.toWCharArray(buffer);
  buffer[name.length()] = 0;
  ULONG nError = RegQueryValueExW(mCurrentKey, buffer, 0, NULL, (LPBYTE)szBuffer, &dwBufferSize);
  QString result;
  if (ERROR_SUCCESS == nError)
    result = QString::fromWCharArray(szBuffer);
  return result;
}

QStringList WbWindowsRegistry::subKeys() const {
  QStringList keys;

  DWORD name;                            // size of name string
  TCHAR className[MAX_PATH] = TEXT("");  // buffer for class name
  DWORD nClassName = MAX_PATH;           // size of class string
  DWORD nSubKeys = 0;                    // number of subkeys
  DWORD maxSubKey;                       // longest subkey size
  DWORD maxClass;                        // longest class string
  DWORD values;                          // number of values for key
  DWORD maxValue;                        // longest value name
  DWORD maxValueData;                    // longest value data
  DWORD securityDescriptor;              // size of security descriptor
  FILETIME lastWriteTime;                // last write time

  // Get the class name and the value count.
  RegQueryInfoKey(mCurrentKey,          // key handle
                  className,            // buffer for class name
                  &nClassName,          // size of class string
                  NULL,                 // reserved
                  &nSubKeys,            // number of subkeys
                  &maxSubKey,           // longest subkey size
                  &maxClass,            // longest class string
                  &values,              // number of values for this key
                  &maxValue,            // longest value name
                  &maxValueData,        // longest value data
                  &securityDescriptor,  // security descriptor
                  &lastWriteTime        // last write time
  );

  // Enumerate the subkeys, until RegEnumKeyEx fails.
  if (nSubKeys) {
    DWORD i;
    for (i = 0; i < nSubKeys; i++) {
      name = MAX_KEY_LENGTH;
      TCHAR subKeyName[MAX_KEY_LENGTH];
      DWORD retCode = RegEnumKeyEx(mCurrentKey, i, subKeyName, &name, NULL, NULL, NULL, &lastWriteTime);
      if (retCode == ERROR_SUCCESS)
        keys << QString(subKeyName);
    }
  }

  return keys;
}
