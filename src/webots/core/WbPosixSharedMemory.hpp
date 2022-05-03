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

// On Linux, we need to use POSIX shared memory segments (shm) and name them snap.webots.*
// to be compliant with the strict confinement policy of snap applications.
// On macOS, POSIX shared memory segments don't have the low limits of the SYSV shared memory segments.
// Unfortunately, the Qt implementation of shared memory segment relies only on SYSV shared memory segments.
// Hence we have to revert to the native POSIX shared memory to be compatible with snap and work around
// macOS limitation with SYSV shared memory.

#ifndef WB_POSIX_SHARED_MEMORY_HPP
#define WB_POSIX_SHARED_MEMORY_HPP

#include <QtCore/QString>

class WbPosixSharedMemory {
public:
  explicit WbPosixSharedMemory(const QString &name);
  ~WbPosixSharedMemory();
  static bool attach() { return false; }
  static bool detach() { return false; }
  bool create(int size);
  int size() const { return mSize; }
  void *data() const { return mData; }
  const QString nativeKey() const { return mName; }

private:
  QString mName;
  int mSize;
  void *mData;
};

#endif
