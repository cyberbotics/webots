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

#ifndef WB_POSIX_MEMORY_MAPPED_FILE_HPP
#define WB_POSIX_MEMORY_MAPPED_FILE_HPP

#include <QtCore/QString>

class WbPosixMemoryMappedFile {
public:
  explicit WbPosixMemoryMappedFile(const QString &name);
  ~WbPosixMemoryMappedFile();
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
