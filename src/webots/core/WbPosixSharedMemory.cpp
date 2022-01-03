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

#include "WbPosixSharedMemory.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

WbPosixSharedMemory::WbPosixSharedMemory(const QString &name) :
#ifdef __APPLE__
  mName(name),
#else
  mName("snap.webots." + name.mid(7)),
#endif
  mSize(0),
  mData(NULL) {
  // we remove the "Webots_" prefix from name and generate a snap compatible POSIX shared memory segment
  shm_unlink(mName.toUtf8());  // delete a possibly existing shared memory segment with the same name
  mFd = shm_open(mName.toUtf8(), O_CREAT | O_RDWR, 0666);  // returns -1 in case of failure
  mSize = 0;
  mData = NULL;
}

WbPosixSharedMemory::~WbPosixSharedMemory() {
  if (mFd < 0)
    return;
  if (mData)
    munmap(mData, mSize);
  shm_unlink(mName.toUtf8());
}

bool WbPosixSharedMemory::create(int size) {
  if (mFd < 0)
    return false;
  if (ftruncate(mFd, size) == -1)
    return false;
  mData = mmap(0, size, PROT_WRITE | PROT_READ, MAP_SHARED, mFd, 0);
  if (mData == MAP_FAILED)
    return false;
  mSize = size;
  return true;
}
