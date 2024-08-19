# Copyright 1996-2023 Cyberbotics Ltd.
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
    wb.wb_supervisor_proto_get_proto_parent.restype = ctypes.c_void_p
    wb.wb_supervisor_proto_get_parameter.restype = ctypes.c_void_p
    wb.wb_supervisor_proto_get_parameter_by_index.restype = ctypes.c_void_p

    def __init__(self, ref: ctypes.c_void_p):
        self._ref = ref

    def get_type_name(self) -> str:
        return self.type_name

    def get_proto_parent(self) -> 'Proto':
        proto = wb.wb_supervisor_proto_get_proto_parent(self._ref)
        return Proto(wb.wb_supervisor_proto_get_proto_parent(self._ref)) if proto else None

    def get_parameter(self, name: str) -> Field:
        field = wb.wb_supervisor_proto_get_parameter(self._ref, str.encode(name))
        return Field(field) if field else None

    def get_parameter_by_index(self, index: int) -> Field:
        field = wb.wb_supervisor_proto_get_parameter_by_index(self._ref, index)
        return Field(field) if field else None

    def get_proto_number_of_parameters(self) -> int:
        return self.number_of_parameters

    @property
    def type_name(self) -> str:
        return wb.wb_supervisor_proto_get_type_name(self._ref).decode()

    @property
    def number_of_parameters(self) -> int:
        return wb.wb_supervisor_proto_get_number_of_parameters(self._ref)
