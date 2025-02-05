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
from .sensor import Sensor
from .radar_target import RadarTarget
from .wb import wb
from typing import List, Union


class Radar(Sensor):
    wb.wb_radar_get_max_range.restype = ctypes.c_double
    wb.wb_radar_get_min_range.restype = ctypes.c_double
    wb.wb_radar_get_horizontal_fov.restype = ctypes.c_double
    wb.wb_radar_get_vertical_fov.restype = ctypes.c_double
    wb.wb_radar_get_targets.restype = ctypes.POINTER(ctypes.c_double)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_radar_enable
        self._get_sampling_period = wb.wb_radar_get_sampling_period
        super().__init__(name, sampling_period)

    def getMaxRange(self) -> float:
        return self.max_range

    def getMinRange(self) -> float:
        return self.min_range

    def getHorizontalFov(self) -> float:
        return self.horizontal_fov

    def getVerticalFov(self) -> float:
        return self.vertical_fov

    def getNumberOfTargets(self) -> int:
        return self.number_of_targets

    def getTargets(self) -> List[RadarTarget]:
        number_of_targets = self.number_of_targets
        data = wb.wb_radar_get_targets(self._tag)
        list = []
        for i in range(number_of_targets):
            list.append(RadarTarget(data[0 + 4*i], data[1 + 4*i], data[2 + 4*i], data[3 + 4*i]))
        return list

    @property
    def max_range(self) -> float:
        return wb.wb_radar_get_max_range(self._tag)

    @property
    def min_range(self) -> float:
        return wb.wb_radar_get_min_range(self._tag)

    @property
    def horizontal_fov(self) -> float:
        return wb.wb_radar_get_horizontal_fov(self._tag)

    @property
    def vertical_fov(self) -> float:
        return wb.wb_radar_get_vertical_fov(self._tag)

    @property
    def number_of_targets(self) -> int:
        return wb.wb_radar_get_number_of_targets(self._tag)
