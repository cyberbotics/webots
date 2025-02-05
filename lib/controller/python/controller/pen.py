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
from .device import Device
import ctypes
from typing import Union


class Pen(Device):
    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def write(self, write: bool):
        wb.wb_pen_write(self._tag, 1 if write else 0)

    def setInkColor(self, color: int, density: float):
        wb.wb_pen_set_ink_color(self._tag, color, ctypes.c_double(density))
