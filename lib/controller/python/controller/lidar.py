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
from .sensor import Sensor
from .lidar_point import LidarPoint
from .wb import wb
from typing import List, Union


class Lidar(Sensor):
    wb.wb_lidar_get_fov.restype = ctypes.c_double
    wb.wb_lidar_get_frequency.restype = ctypes.c_double
    wb.wb_lidar_get_vertical_fov.restype = ctypes.c_double
    wb.wb_lidar_get_max_frequency.restype = ctypes.c_double
    wb.wb_lidar_get_min_frequency.restype = ctypes.c_double
    wb.wb_lidar_get_max_range.restype = ctypes.c_double
    wb.wb_lidar_get_min_range.restype = ctypes.c_double
    wb.wb_lidar_get_point_cloud.restype = ctypes.POINTER(ctypes.c_ubyte)
    wb.wb_lidar_get_layer_point_cloud.restype = ctypes.POINTER(ctypes.c_ubyte)
    wb.wb_lidar_get_range_image.restype = ctypes.POINTER(ctypes.c_float)
    wb.wb_lidar_get_layer_range_image.restype = ctypes.POINTER(ctypes.c_float)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_lidar_enable
        self._get_sampling_period = wb.wb_lidar_get_sampling_period
        super().__init__(name, sampling_period)

    def getFov(self) -> float:
        return self.fov

    def getVerticalFov(self) -> float:
        return self.vertical_fov

    def getMaxFrequency(self) -> float:
        return self.max_frequency

    def getMinFrequency(self) -> float:
        return self.min_frequency

    def getMaxRange(self) -> float:
        return self.max_range

    def getMinRange(self) -> float:
        return self.min_range

    def getHorizontalResolution(self) -> int:
        return self.horizontal_resolution

    def getNumberOfLayers(self) -> int:
        return self.number_of_layers

    def getFrequency(self) -> float:
        return self.frequency

    def setFrequency(self, frequency: float):
        self.frequency = frequency

    def getNumberOfPoints(self) -> int:
        return self.number_of_points

    def getRangeImage(self) -> List[float]:
        return self.range_image[:self.horizontal_resolution * self.number_of_layers]

    def getRangeImageArray(self) -> List[List[float]]:
        array = []
        for i in range(self.number_of_layers):
            array.append(self.getLayerRangeImage(i))
        return array

    def getLayerRangeImage(self, layer) -> List[float]:
        return wb.wb_lidar_get_range_image(self._tag, layer)[:self.horizontal_resolution]

    @property
    def range_image(self):
        return wb.wb_lidar_get_range_image(self._tag)

    @property
    def fov(self) -> float:
        return wb.wb_lidar_get_fov(self._tag)

    @property
    def vertical_fov(self) -> float:
        return wb.wb_lidar_get_vertical_fov(self._tag)

    @property
    def max_frequency(self) -> float:
        return wb.wb_lidar_get_max_frequency(self._tag)

    @property
    def min_frequency(self) -> float:
        return wb.wb_lidar_get_min_frequency(self._tag)

    @property
    def max_range(self) -> float:
        return wb.wb_lidar_get_max_range(self._tag)

    @property
    def min_range(self) -> float:
        return wb.wb_lidar_get_min_range(self._tag)

    @property
    def horizontal_resolution(self) -> int:
        return wb.wb_lidar_get_horizontal_resolution(self._tag)

    @property
    def number_of_layers(self) -> int:
        return wb.wb_lidar_get_number_of_layers(self._tag)

    @property
    def frequency(self) -> float:
        return wb.wb_lidar_get_frequency(self._tag)

    @frequency.setter
    def frequency(self, frequency: float):
        wb.wb_lidar_set_frequency(self._tag, ctypes.c_double(frequency))

    @property
    def number_of_points(self) -> int:
        return wb.wb_lidar_get_number_of_points(self._tag)

    def disablePointCloud(self):
        wb.wb_lidar_disable_point_cloud(self._tag)

    def enablePointCloud(self):
        wb.wb_lidar_enable_point_cloud(self._tag)

    def isPointCloudEnabled(self) -> bool:
        return wb.wb_lidar_is_point_cloud_enabled(self._tag) != 0

    def getPointCloud(self) -> List[LidarPoint]:
        number_of_points = self.number_of_points
        data = bytes(wb.wb_lidar_get_point_cloud(self._tag)[:number_of_points * 20])
        list = []
        for i in range(number_of_points):
            list.append(LidarPoint(data, i))
        return list

    def getLayerPointCloud(self, layer: int) -> List[LidarPoint]:
        number_of_points = int(self.number_of_points / self.number_of_layers)
        data = bytes(wb.wb_lidar_get_layer_point_cloud(self._tag, layer)[:number_of_points * 20])
        list = []
        for i in range(number_of_points):
            list.append(LidarPoint(data, i))
        return list
