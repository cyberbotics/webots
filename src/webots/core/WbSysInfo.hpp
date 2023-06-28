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

#ifndef WB_SYS_INFO_HPP
#define WB_SYS_INFO_HPP

#include <QtCore/QString>

class QOpenGLFunctions;

namespace WbSysInfo {
  enum WbPlatform { LINUX_PLATFORM, MACOS_PLATFORM, WIN32_PLATFORM };

  const QString &sysInfo();
  const QString &platformShortName();  // either "windows", "mac", "linux32" or "linux64"
  const QString &processor();
  bool isVirtualMachine();
#ifdef __linux__
  const QString &linuxCpuModelName();
  bool isRootUser();
  inline bool isSnap() {
    return qgetenv("SNAP_NAME") == "webots";
  }
#else
  inline bool isSnap() {
    return false;
  }
#endif
  QString environmentVariable(const QString &name);
  void setEnvironmentVariable(const QString &name, const QString &value);
  QString shortPath(const QString &path);  // Windows 8.3 short path name

  WbPlatform platform();

  bool isPointerSize32bits();
  bool isPointerSize64bits();

  int coreCount();

#ifdef _WIN32
  quint32 gpuDeviceId(QOpenGLFunctions *gl);
  quint32 gpuVendorId(QOpenGLFunctions *gl);
  int intelGPUGeneration(QOpenGLFunctions *gl);
  bool isAmdLowEndGpu(QOpenGLFunctions *gl);
#else
  bool isLowEndGpu();
#endif
  const void initializeOpenGlInfo();
  const QString &openGLRenderer();
  const QString &openGLVendor();
  const QString &openGLVersion();
  void openGlLineWidthRange(double &min, double &max);
};  // namespace WbSysInfo

#endif
