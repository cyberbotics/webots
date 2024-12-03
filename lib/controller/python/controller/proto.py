# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes
from .wb import wb
from .field import Field


class Proto:
    wb.wb_supervisor_proto_get_type_name.restype = ctypes.c_char_p
    wb.wb_supervisor_proto_get_parent.restype = ctypes.c_void_p
    wb.wb_supervisor_proto_get_field.restype = ctypes.c_void_p
    wb.wb_supervisor_proto_get_field_by_index.restype = ctypes.c_void_p

    def __init__(self, ref: ctypes.c_void_p):
        self._ref = ref

    def getTypeName(self) -> str:
        return self.type_name

    def getParent(self) -> 'Proto':
        proto = wb.wb_supervisor_proto_get_parent(self._ref)
        return Proto(proto) if proto else None

    def getField(self, name: str) -> Field:
        field = wb.wb_supervisor_proto_get_field(self._ref, str.encode(name))
        return Field(field) if field else None

    def getFieldByIndex(self, index: int) -> Field:
        field = wb.wb_supervisor_proto_get_field_by_index(self._ref, index)
        return Field(field) if field else None

    def getNumberOfFields(self) -> int:
        return self.number_of_fields

    def isDerived(self) -> bool:
        return self.is_derived

    @property
    def type_name(self) -> str:
        return wb.wb_supervisor_proto_get_type_name(self._ref).decode()

    @property
    def number_of_fields(self) -> int:
        return wb.wb_supervisor_proto_get_number_of_fields(self._ref)

    @property
    def is_derived(self) -> bool:
        return wb.wb_supervisor_proto_is_derived(self._ref) != 0
