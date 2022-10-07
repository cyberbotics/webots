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
from controller.constants import constant
from controller.device import Device
from typing import Union


class Motor(Device):
    ROTATIONAL = constant('ROTATIONAL')
    LINEAR = constant('LINEAR')

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    wb.wb_motor_get_target_position.restype = ctypes.c_double

    def setPosition(self, p: float):
        wb.wb_motor_set_position(self._tag, ctypes.c_double(p))

    def setVelocity(self, v: float):
        wb.wb_motor_set_velocity(self._tag, ctypes.c_double(v))

    def getType(self) -> int:
        return wb.wb_motor_get_type(self._tag)

    @property
    def target_position(self) -> float:
        return wb.wb_motor_get_target_position(self._tag)

    @target_position.setter
    def target_position(self, p: float):
        wb.wb_motor_set_position(self._tag, ctypes.c_double(p))

    wb.wb_motor_get_velocity.restype = ctypes.c_double

    @property
    def target_velocity(self) -> float:
        return wb.wb_motor_get_velocity(self._tag)

    @target_velocity.setter
    def target_velocity(self, v: float):
        wb.wb_motor_set_velocity(self._tag, ctypes.c_double(v))

    @property
    def type(self) -> int:
        return wb.wb_motor_get_type(self._tag)
