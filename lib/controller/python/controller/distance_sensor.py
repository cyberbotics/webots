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

import ctypes
from .wb import wb
from .sensor import Sensor
from typing import Union, List


class DistanceSensor(Sensor):
    GENERIC = 0
    INFRA_RED = 1
    SONAR = 2
    LASER = 3
    wb.wb_distance_sensor_get_aperture.restype = ctypes.c_double
    wb.wb_distance_sensor_get_max_value.restype = ctypes.c_double
    wb.wb_distance_sensor_get_min_value.restype = ctypes.c_double
    wb.wb_distance_sensor_get_value.restype = ctypes.c_double
    wb.wb_distance_sensor_get_lookup_table.restype = ctypes.POINTER(ctypes.c_double)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_distance_sensor_enable
        self._get_sampling_period = wb.wb_distance_sensor_get_sampling_period
        super().__init__(name, sampling_period)

    def getAperture(self) -> float:
        return self.aperture

    def getLookupTable(self) -> List[float]:
        return self.lookup_table[:3 * wb.wb_distance_sensor_get_lookup_table_size(self._tag)]

    def getMaxValue(self) -> float:
        return self.max_value

    def getMinValue(self) -> float:
        return self.min_value

    def getType(self) -> int:
        return self.type

    def getValue(self) -> float:
        return self.value

    @property
    def aperture(self) -> float:
        return wb.wb_distance_sensor_get_aperture(self._tag)

    @property
    def lookup_table(self):
        return wb.wb_distance_sensor_get_lookup_table(self._tag)

    @property
    def max_value(self) -> float:
        return wb.wb_distance_sensor_get_max_value(self._tag)

    @property
    def min_value(self) -> float:
        return wb.wb_distance_sensor_get_min_value(self._tag)

    @property
    def type(self) -> int:
        return wb.wb_distance_sensor_get_type(self._tag)

    @property
    def value(self) -> float:
        return wb.wb_distance_sensor_get_value(self._tag)
