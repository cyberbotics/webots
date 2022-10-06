# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes
from controller.wb import wb
from controller import Field


class Node:
    wb.wb_supervisor_node_get_from_def.restype = ctypes.c_void_p

    def __init__(self, DEF=None):
        if DEF:
            self._ref = wb.wb_supervisor_node_get_from_def(str.encode(DEF))

    def getField(self, name):
        return Field(self, name)

    wb.wb_supervisor_node_get_type_name.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_node_get_type_name.restype = ctypes.c_char_p

    @property
    def type_name(self) -> str:
        return wb.wb_supervisor_node_get_type_name(self._ref).decode()
