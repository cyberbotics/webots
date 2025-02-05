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

from .sensor import Sensor
from .wb import wb
import ctypes
import typing


class GPS(Sensor):
    LOCAL = 0
    WGS84 = 1

    wb.wb_gps_get_speed.restype = ctypes.c_double
    wb.wb_gps_get_speed_vector.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_gps_get_values.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_gps_convert_to_degrees_minutes_seconds.restype = ctypes.c_char_p

    def __init__(self, name: typing.Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_gps_enable
        self._get_sampling_period = wb.wb_gps_get_sampling_period
        super().__init__(name, sampling_period)

    def getCoordinateSystem(self) -> int:
        return self.coordinate_system

    @property
    def coordinate_system(self) -> int:
        return wb.wb_gps_get_coordinate_system(self._tag)

    @staticmethod
    def convertToDegreesMinutesSeconds(decimalDegrees) -> str:
        degrees = int(decimalDegrees)
        minutes = int(decimalDegrees - degrees) * 60
        seconds = int(((decimalDegrees - degrees) * 60) - minutes) * 60
        return f'{degrees}° {minutes}′ {seconds}″'

    def getSpeed(self) -> float:
        return self.speed

    @property
    def speed(self) -> float:
        return wb.wb_gps_get_speed(self._tag)

    def getSpeedVector(self) -> typing.List[float]:
        return self.speed_vector

    @property
    def speed_vector(self) -> typing.List[float]:
        return wb.wb_gps_get_speed_vector(self._tag)[:3]

    def getValues(self) -> typing.List[float]:
        return self.value

    @property
    def value(self) -> typing.List[float]:
        return wb.wb_gps_get_values(self._tag)[:3]
