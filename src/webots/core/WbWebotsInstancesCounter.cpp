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

#include "WbWebotsInstancesCounter.hpp"

#include "WbStandardPaths.hpp"

#include <QtCore/QString>

#ifdef _WIN32
#include <windows.h>

#include <tchar.h>
#include <tlhelp32.h>
#elif defined(__linux__)
#include <QtCore/QDir>
#include <QtCore/QTextStream>
#elif defined(__APPLE__)
#include <libproc.h>
#include <strings.h>
#include <sys/proc_info.h>
#else
#include <cassert.h>
#endif

int WbWebotsInstancesCounter::numberOfInstances() {
  int numberOfWebotsInstances = 0;
#ifdef _WIN32
  // idea use the Windows process snapshot
  // cf. https://msdn.microsoft.com/en-us/library/ms686701(VS.85).aspx
  HANDLE hProcessSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
  if (hProcessSnap == INVALID_HANDLE_VALUE)
    return 1;

  PROCESSENTRY32 pe32;
  pe32.dwSize = sizeof(PROCESSENTRY32);
  if (!Process32First(hProcessSnap, &pe32)) {
    CloseHandle(hProcessSnap);
    return 1;
  }

  do {
    QString processName = (LPSTR)pe32.szExeFile;
    if (processName.trimmed() == "webots.exe")
      ++numberOfWebotsInstances;
  } while (Process32Next(hProcessSnap, &pe32));

  CloseHandle(hProcessSnap);
#elif defined(__linux__)
  // Idea: the number of `/proc/[0-9]/comm` files containing exactly `webots-bin`
  //       is matching with numberOfWebotsInstances.
  //       Moreover the `/proc/[0-9]/status` files are used to determine if the
  //       instance is indeed running (or sleeping), but not defect
  // cf. http://stackoverflow.com/questions/939778/linux-api-to-list-running-processes
  QDir processesDir("/proc");
  if (!processesDir.exists())
    return 1;

  processesDir.setFilter(QDir::Dirs | QDir::NoSymLinks);
  QStringList filters;
  filters << "[0-9]*";
  processesDir.setNameFilters(filters);

  QFileInfoList dirList = processesDir.entryInfoList();
  for (int i = 0; i < dirList.size(); ++i) {
    QDir processDir = QDir(dirList.at(i).absoluteFilePath());
    QFileInfo commandFileInfo(processDir, "comm");
    QFileInfo statusFileInfo(processDir, "status");
    if (commandFileInfo.exists() && statusFileInfo.exists()) {
      QFile commandFile(commandFileInfo.absoluteFilePath());
      QFile statusFile(statusFileInfo.absoluteFilePath());
      if (!commandFile.open(QFile::ReadOnly | QFile::Text) || !statusFile.open(QFile::ReadOnly | QFile::Text))
        break;

      QTextStream in;
      in.setDevice(&commandFile);
      QString command = in.readAll().trimmed();
      commandFile.close();
      in.setDevice(&statusFile);
      QString status = in.readAll().trimmed();
      statusFile.close();

      if (command == "webots-bin" && status.contains(QRegExp("State:\\s*[RS]")))
        ++numberOfWebotsInstances;
    }
  }
#elif defined(__APPLE__)
  // Idea:
  //   use libproc.h ant its info structure to list the processes and its state (running)
  // cf. http://stackoverflow.com/questions/3018054/retrieve-names-of-running-processes
  int numberOfProcesses = proc_listpids(PROC_ALL_PIDS, 0, NULL, 0);
  pid_t pids[numberOfProcesses];
  bzero(pids, sizeof(pids));
  proc_listpids(PROC_ALL_PIDS, 0, pids, sizeof(pids));
  for (int i = 0; i < numberOfProcesses; ++i) {
    if (pids[i] == 0)
      continue;
    char pathBuffer[PROC_PIDPATHINFO_MAXSIZE];
    bzero(pathBuffer, PROC_PIDPATHINFO_MAXSIZE);
    proc_pidpath(pids[i], pathBuffer, sizeof(pathBuffer));

    if (strlen(pathBuffer) > 0 && QString(pathBuffer).endsWith("Contents/MacOS/webots")) {
      struct proc_bsdshortinfo bsdinfo;
      int numbytes = proc_pidinfo(pids[i], PROC_PIDT_SHORTBSDINFO, (uint64_t)0, &bsdinfo, PROC_PIDT_SHORTBSDINFO_SIZE);
      if (numbytes > 0 && bsdinfo.pbsi_status == 2 /*SRUN*/)
        ++numberOfWebotsInstances;
    }
  }
#else
  assert(0);  // unknown OS
#endif

  // if an issue has occured, return a coherent value
  if (numberOfWebotsInstances < 1)
    return 1;

  return numberOfWebotsInstances;
}
