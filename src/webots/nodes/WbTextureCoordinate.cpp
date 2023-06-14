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

#include "WbTextureCoordinate.hpp"
#include "WbMFVector2.hpp"

void WbTextureCoordinate::init() {
  mPoint = findMFVector2("point");
}

WbTextureCoordinate::WbTextureCoordinate(WbTokenizer *tokenizer) : WbBaseNode("TextureCoordinate", tokenizer) {
  init();
}

WbTextureCoordinate::WbTextureCoordinate(const WbTextureCoordinate &other) : WbBaseNode(other) {
  init();
}

WbTextureCoordinate::WbTextureCoordinate(const WbNode &other) : WbBaseNode(other) {
  init();
}

WbTextureCoordinate::~WbTextureCoordinate() {
}

QStringList WbTextureCoordinate::fieldsToSynchronizeWithX3D() const {
  QStringList fields;
  fields << "point";
  return fields;
}
