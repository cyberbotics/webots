// Copyright 1996-2021 Cyberbotics Ltd.
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

#ifndef UPLOADER_HPP
#define UPLOADER_HPP

#include <string>

class Serial;

class Uploader {
public:
  static int connect(const std::string &port);
  static void disconnect();
  static int send(int robot_id, const std::string &hexfilename, void (*updateProgressFunction)(int, int));
  static void cancelUpload();

private:
  static Serial *cSerial;
};

#endif  // UPLOADER_HPP
