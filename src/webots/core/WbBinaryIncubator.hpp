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

#ifndef WB_BINARY_INCUBATOR_HPP
#define WB_BINARY_INCUBATOR_HPP

//
// Description: Helper class to copy a binary file and its dependencies from the build directory
//

#include <QtCore/QObject>

class WbBinaryIncubator : public QObject {
public:
  enum Error {
    NONE = 0,
    INVALID_BINARY_PATH,
    INVALID_BINARY_TYPE,
    UNEXISTING_BUILD_FOLDER,
    FILE_REMOVE_ERROR,
    FILE_COPY_ERROR
  };

  // filename example: /path/controllers/my_controller/my_controller.exe
  // return success
  static Error copyBinaryAndDependencies(const QString &filename);

private:
  WbBinaryIncubator() {}  // static class

  static Error copyBinaryPath(const QString &path);
};

#endif
