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

#define WB_ALLOW_MIXING_C_AND_CPP_API
#include <webots/supervisor.h>
#include <webots/Proto.hpp>

#include <stdio.h>
#include <map>

using namespace std;
using namespace webots;

static map<WbProtoRef, Proto *> protoMap;

Proto *Proto::findProto(WbProtoRef ref) {
  if (!ref)
    return NULL;

  map<WbProtoRef, Proto *>::iterator iter = protoMap.find(ref);
  if (iter != protoMap.end())
    return iter->second;

  Proto *proto = new Proto(ref);
  protoMap.insert(pair<WbProtoRef, Proto *>(ref, proto));
  return proto;
}

void Proto::cleanup() {
  map<WbProtoRef, Proto *>::iterator iter;
  for (iter = protoMap.begin(); iter != protoMap.end(); ++iter)
    delete (*iter).second;

  protoMap.clear();
}

Proto::Proto(WbProtoRef ref) : protoRef(ref) {
}

string Proto::getTypeName() const {
  return wb_supervisor_proto_get_type_name(protoRef);
}

bool Proto::isDerived() const {
  return wb_supervisor_proto_is_derived(protoRef);
}

Proto *Proto::getParent() const {
  return findProto(wb_supervisor_proto_get_parent(protoRef));
}

Field *Proto::getField(const string &fieldName) const {
  return Field::findField(wb_supervisor_proto_get_field(protoRef, fieldName.c_str()));
}

Field *Proto::getFieldByIndex(const int index) const {
  return Field::findField(wb_supervisor_proto_get_field_by_index(protoRef, index));
}

int Proto::getNumberOfFields() const {
  return wb_supervisor_proto_get_number_of_fields(protoRef);
}
