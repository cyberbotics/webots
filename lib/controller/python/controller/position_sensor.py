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
from .wb import wb
from .sensor import Sensor
from typing import Union


class PositionSensor(Sensor):
    ROTATIONAL = constant('ROTATIONAL')
    LINEAR = constant('LINEAR')

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_position_sensor_enable
        self._get_sampling_period = wb.wb_position_sensor_get_sampling_period
        super().__init__(name, sampling_period)

    wb.wb_position_sensor_get_value.restype = ctypes.c_double

    def getValue(self) -> float:
        return self.value

    def getBrake(self):
        return self.brake

    def getMotor(self):
        return self.motor

    def getType(self) -> int:
        return self.type

    @property
    def brake(self):
        from .brake import Brake
        tag = wb.wb_position_sensor_get_brake(self._tag)
        return None if tag == 0 else Brake(tag)

    @property
    def motor(self):
        from .motor import Motor
        tag = wb.wb_brake_get_motor(self._tag)
        return None if tag == 0 else Motor(tag)

    @property
    def value(self) -> float:
        return wb.wb_position_sensor_get_value(self._tag)

    @property
    def type(self) -> int:
        return wb.wb_position_sensor_get_type(self._tag)
