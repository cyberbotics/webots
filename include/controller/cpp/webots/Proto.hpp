// Copyright 1996-2024 Cyberbotics Ltd.
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

#ifndef PROTO_HPP
#define PROTO_HPP

#define WB_USING_CPP_API
#include <string>
#include <webots/Field.hpp>
#include "../../c/webots/types.h"

namespace webots {
  class Field;
  class Proto {
  public:
    std::string getTypeName() const;
    bool isDerived() const;
    Proto *getParent() const;
    Field *getField(const std::string &fieldName) const;
    Field *getFieldByIndex(const int index) const;
    int getNumberOfFields() const;

    // DO NOT USE THESE FUNCTIONS: THEY ARE RESERVED FOR INTERNAL USE:
    static Proto *findProto(WbProtoRef ref);
    static void cleanup();

  private:
    Proto(WbProtoRef ref);
    ~Proto() {}

    WbProtoRef protoRef;
  };
}  // namespace webots

#endif  // PROTO_HPP
