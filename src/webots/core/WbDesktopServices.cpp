// Copyright 1996-2019 Cyberbotics Ltd.
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

#include "WbDesktopServices.hpp"

#ifdef __linux__
#include <QtCore/QProcess>
#else
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#endif

bool WbDesktopServices::openUrl(const QString &url) {
#ifdef __linux__
  QProcess process;
  process.setProgram("xdg-open");  // we need to use xdg-open to be snap compliant
  process.setArguments(QStringList() << url);
  process.setStandardErrorFile(QProcess::nullDevice())
  process.setStandardOutputFile(QProcess::nullDevice());
  return process.startDetached();
#else
  return QDesktopServices::openUrl(QUrl(url));
#endif
}
