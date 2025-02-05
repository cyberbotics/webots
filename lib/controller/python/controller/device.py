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

from .wb import wb
import ctypes
from typing import Union


class Device:
    wb.wb_device_get_name.restype = ctypes.c_char_p
    wb.wb_device_get_model.restype = ctypes.c_char_p

    def __init__(self, name: Union[str, int]):
        self._tag = name if isinstance(name, int) else wb.wb_robot_get_device(str.encode(name))

    def getName(self) -> str:
        return self.name

    def getModel(self) -> str:
        return self.model

    def getNodeType(self) -> int:
        return self.node_type

    @property
    def name(self) -> str:
        return wb.wb_device_get_name(self._tag).decode()

    @property
    def model(self) -> str:
        return wb.wb_device_get_model(self._tag).decode()

    @property
    def node_type(self) -> int:
        return wb.wb_device_get_node_type(self._tag)
