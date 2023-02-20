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

#ifndef WB_WINDOWS_REGISTRY_HPP
#define WB_WINDOWS_REGISTRY_HPP

#ifndef _WIN32
#error "Unsupported OS"
#endif

#include <QtCore/QStringList>

#include <windows.h>

//
// Description: query the Windows registry
//

class WbWindowsRegistry {
public:
  explicit WbWindowsRegistry(const QString &key);  // e.g. key = "\\HKEY_CURRENT_USER\\Software\\Cyberbotics"
  ~WbWindowsRegistry();
  QString stringValue(const QString &name) const;
  QStringList subKeys() const;  // retrieve the list of subKeys (but not the values), e.g. "Webots8.5"

private:
  HKEY mCurrentKey;
};

#endif
