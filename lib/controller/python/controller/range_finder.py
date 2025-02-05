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
from .wb import wb
from typing import List, Union


class RangeFinder(Sensor):
    wb.wb_range_finder_get_fov.restype = ctypes.c_double
    wb.wb_range_finder_get_max_range.restype = ctypes.c_double
    wb.wb_range_finder_get_min_range.restype = ctypes.c_double
    wb.wb_range_finder_get_range_image.restype = ctypes.POINTER(ctypes.c_float)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_range_finder_enable
        self._get_sampling_period = wb.wb_range_finder_get_sampling_period
        super().__init__(name, sampling_period)

    def getFov(self) -> float:
        return self.fov

    def getWidth(self) -> int:
        return self.width

    def getHeight(self) -> int:
        return self.height

    def getMaxRange(self) -> float:
        return self.max_range

    def getMinRange(self) -> float:
        return self.min_range

    def getRangeImage(self, data_type='list') -> List[float]:
        if data_type == 'list':
            return self.range_image[:self.width * self.height]
        else:
            return self.range_image

    def getRangeImageArray(self) -> List[List[float]]:
        range_image = self.range_image
        width = self.width
        array = []
        for i in range(self.height):
            array.append(range_image[i * width:(i + 1) * width])
        return array

    @staticmethod
    def rangeImageGetDepth(image, width, x, y) -> float:
        return image[y * width + x]

    def saveImage(self, filename: str, quality: int) -> int:
        return wb.wb_range_finder_save_image(self._tag, str.encode(filename), quality)

    @property
    def range_image(self):
        return wb.wb_range_finder_get_range_image(self._tag)

    @property
    def fov(self) -> float:
        return wb.wb_range_finder_get_fov(self._tag)

    @property
    def width(self) -> int:
        return wb.wb_range_finder_get_width(self._tag)

    @property
    def height(self) -> int:
        return wb.wb_range_finder_get_height(self._tag)

    @property
    def max_range(self) -> float:
        return wb.wb_range_finder_get_max_range(self._tag)

    @property
    def min_range(self) -> float:
        return wb.wb_range_finder_get_min_range(self._tag)
