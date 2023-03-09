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
from .constants import constant
from .device import Device
from .wb import wb
from typing import Union


class Brake(Device):
    ROTATIONAL = constant('ROTATIONAL')
    LINEAR = constant('LINEAR')

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def setDampingConstant(self, c: float):
        wb.wb_brake_set_damping_constant(self._tag, ctypes.c_double(c))

    dampingConstant = property(fset=setDampingConstant)

    def getType(self) -> int:
        return self.type

    def getMotor(self):
        return self.motor

    def getPositionSensor(self):
        return self.position_sensor

    @property
    def motor(self):
        from .motor import Motor
        tag = wb.wb_brake_get_motor(self._tag)
        return None if tag == 0 else Motor(tag)

    @property
    def position_sensor(self):
        from .position_sensor import PositionSensor
        tag = wb.wb_brake_get_position_sensor(self._tag)
        return None if tag == 0 else PositionSensor(tag)

    @property
    def type(self) -> int:
        return wb.wb_brake_get_type(self._tag)
