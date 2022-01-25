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

#include "WbDesktopServices.hpp"
#include "WbLog.hpp"

#include <QtCore/QProcess>
#ifndef __linux__
#include <QtCore/QUrl>
#include <QtGui/QDesktopServices>
#endif

bool WbDesktopServices::openUrl(const QString &url) {
#ifdef __linux__
  QProcess process;
  process.setProgram("xdg-open");  // we need to use xdg-open to be snap compliant
  process.setArguments(QStringList() << url);
  process.setStandardErrorFile(QProcess::nullDevice());
  process.setStandardOutputFile(QProcess::nullDevice());
  return process.startDetached();
#else
#ifdef _WIN32
  // The TEMP/TMP environment variables set my the MSYS2 console are confusing Visual C++ (among possibly other apps)
  // as they refer to "/tmp" which is not a valid Windows path. It is therefore safer to remove them
  const QByteArray TEMP = qgetenv("TEMP");
  const QByteArray TMP = qgetenv("TMP");
  qunsetenv("TMP");
  qunsetenv("TEMP");
#endif
  bool result = QDesktopServices::openUrl(QUrl(url));
#ifdef _WIN32
  qputenv("TEMP", TEMP);
  qputenv("TMP", TMP);
#endif
  return result;
#endif
}

bool WbDesktopServices::openUrlWithArgs(const QString &url, const QString &program, const bool newBrowserWindow) {
  QString systemProgram;
  QStringList arguments;

#ifdef _WIN32
  // The TEMP/TMP environment variables set my the MSYS2 console are confusing Visual C++ (among possibly other apps)
  // as they refer to "/tmp" which is not a valid Windows path. It is therefore safer to remove them
  const QByteArray TEMP = qgetenv("TEMP");
  const QByteArray TMP = qgetenv("TMP");
  qunsetenv("TMP");
  qunsetenv("TEMP");
  if (program.isEmpty())
    return openUrl(url);

  systemProgram = "start" + program;
  arguments << program;
#elif __linux__
  if (program.isEmpty())
    return openUrl(url);

  systemProgram = program;
#else
  if (program.isEmpty())
    return openUrl(url);

  systemProgram = "open";  // set argument
  arguments << "-a " + program;
#endif

  QProcess process;
  process.setProgram(systemProgram);
  if (newBrowserWindow)
    process.setArguments(arguments << "-new-window" << url);
  else
    process.setArguments(arguments << url);
  process.setStandardErrorFile(QProcess::nullDevice());
  process.setStandardOutputFile(QProcess::nullDevice());
  bool result = process.startDetached();
  if (!result) {
    WbLog::warning(QObject::tr("Failed to open web browser: %1. Open robot window in default browser.").arg(program));
    result = openUrl(url);
  }
#ifdef _WIN32
  qputenv("TEMP", TEMP);
  qputenv("TMP", TMP);
#endif
  return result;
}
