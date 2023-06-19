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

from .device import Device
from .wb import wb
import ctypes
from typing import List, Union


class Skin(Device):
    wb.wb_skin_get_bone_name.restype = ctypes.c_char_p
    wb.wb_skin_get_bone_orientation.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_skin_get_bone_position.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_skin_set_bone_orientation.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_char]
    wb.wb_skin_set_bone_position.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_char]

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def getBoneCount(self) -> int:
        return self.bone_count

    def getBoneName(self, index: int) -> str:
        return wb.wb_skin_get_bone_name(self._tag, index).decode()

    def getBoneOrientation(self, index: int, absolute: bool) -> List[float]:
        return wb.wb_skin_get_bone_orientation(self._tag, index, 1 if absolute else 0)

    def getBonePosition(self, index: int, absolute: bool) -> List[float]:
        return wb.wb_skin_get_bone_position(self._tag, index, 1 if absolute else 0)

    def setBoneOrientation(self, index: int, orientation: List[float], absolute: bool):
        data = (ctypes.c_double * 4)(orientation[0], orientation[1], orientation[2], orientation[3])
        wb.wb_skin_set_bone_orientation(self._tag, index, data, 1 if absolute else 0)

    def setBonePosition(self, index: int, position: List[float], absolute: bool):
        data = (ctypes.c_double * 3)(position[0], position[1], position[2])
        wb.wb_skin_set_bone_position(self._tag, index, data, 1 if absolute else 0)

    @property
    def bone_count(self) -> int:
        return wb.wb_skin_get_bone_count(self._tag)
