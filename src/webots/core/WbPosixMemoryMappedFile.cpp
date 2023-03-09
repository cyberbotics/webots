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

#include "WbPosixMemoryMappedFile.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

WbPosixMemoryMappedFile::WbPosixMemoryMappedFile(const QString &name) : mName(name), mSize(0), mData(NULL) {
}

WbPosixMemoryMappedFile::~WbPosixMemoryMappedFile() {
  if (mData)
    munmap(mData, mSize);
  unlink(mName.toUtf8());
}

bool WbPosixMemoryMappedFile::create(int size) {
  unlink(mName.toUtf8());                                 // delete a possibly existing memory mapped file with the same name
  int fd = open(mName.toUtf8(), O_CREAT | O_RDWR, 0666);  // returns -1 in case of failure
  if (fd < 0)
    return false;
  if (ftruncate(fd, size) == -1)
    return false;
  mData = mmap(0, size, PROT_WRITE | PROT_READ, MAP_SHARED, fd, 0);
  close(fd);
  if (mData == MAP_FAILED)
    return false;
  mSize = size;
  return true;
}
