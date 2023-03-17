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

from .sensor import Sensor
from .wb import wb
import ctypes
import typing


class InertialUnit(Sensor):
    wb.wb_inertial_unit_get_roll_pitch_yaw.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_inertial_unit_get_quaternion.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_inertial_unit_get_noise.restype = ctypes.c_double

    def __init__(self, name: typing.Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_inertial_unit_enable
        self._get_sampling_period = wb.wb_inertial_unit_get_sampling_period
        super().__init__(name, sampling_period)

    def getRollPitchYaw(self) -> typing.List[float]:
        return self.roll_pitch_yaw

    def getQuaternion(self) -> typing.List[float]:
        return self.quaternion

    def getNoise(self) -> float:
        return self.noise

    @property
    def roll_pitch_yaw(self) -> typing.List[float]:
        return wb.wb_inertial_unit_get_roll_pitch_yaw(self._tag)[:3]

    @property
    def quaternion(self) -> typing.List[float]:
        return wb.wb_inertial_unit_get_quaternion(self._tag)[:4]

    @property
    def noise(self) -> float:
        return wb.wb_inertial_unit_get_noise(self._tag)
