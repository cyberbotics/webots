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
import struct
from typing import Union


class MouseState():
    def __init__(self, u: float, v: float, x: float, y: float, z: float, left: bool, middle: bool, right: bool):
        self.u = u
        self.v = v
        self.x = x
        self.y = y
        self.z = z
        self.left = left
        self.middle = middle
        self.right = right


class Mouse():
    wb.wb_mouse_get_state_pointer.restype = ctypes.POINTER(ctypes.c_ubyte)

    def __init__(self, sampling_period: int = None):
        if sampling_period != 0:
            self.sampling_period = int(wb.wb_robot_get_basic_time_step()) if sampling_period is None else sampling_period

    def enable(self, p: int):
        wb.wb_mouse_enable(p)

    def disable(self):
        wb.wb_mouse_disable()

    def getSamplingPeriod(self) -> int:
        return self.sampling_period

    def getState(self) -> MouseState:
        data = bytes(wb.wb_mouse_get_state_pointer()[0:43])
        values = struct.unpack('5d3?', data)
        return MouseState(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7])

    def enable3dPosition(self):
        wb.wb_mouse_enable_3d_position()

    def disable3dPosition(self):
        wb.wb_mouse_disable_3d_position()

    def is3dPositionEnabled(self) -> bool:
        return wb.wb_mouse_is_3d_position_enabled() != 0

    @property
    def sampling_period(self) -> int:
        return wb.wb_mouse_get_sampling_period()

    @sampling_period.setter
    def sampling_period(self, p: Union[int, None]):
        if p is None:
            p = 0
        wb.wb_mouse_enable(p)
