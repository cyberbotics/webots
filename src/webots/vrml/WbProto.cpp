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

#include "WbProto.hpp"

#include "WbField.hpp"
#include "WbProtoModel.hpp"

WbProto::WbProto(WbProto *other, WbNode *node) : mNode(node) {
  mParentProto = other->mParentProto;
  mModel = other->mModel;
  mModel->ref();
  foreach (const WbField *parameter, other->parameters()) {
    WbField *copy = new WbField(*parameter, other->mNode);
    mParameters.append(copy);
  }
}

WbProto::~WbProto() {
  // Delete parameters backwards to always delete USE nodes before DEF nodes
  int n = mParameters.size() - 1;
  for (int i = n; i >= 0; --i)
    delete mParameters[i];
  mModel->unref();
}

const QString &WbProto::name() const {
  return mModel->name();
}

int WbProto::parameterIndex(const WbField *field) const {
  const QList<WbField *> &parameterList = parameters();
  return parameterList.indexOf(const_cast<WbField *>(field));
}
