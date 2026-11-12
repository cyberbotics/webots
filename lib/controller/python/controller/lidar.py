# Copyright 1996-2025 Cyberbotics Ltd.
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
import sys
from .sensor import Sensor
from .lidar_point import LidarPoint
from .wb import wb
from typing import List, Optional, Union


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

    def getRangeImage(self) -> Optional[List[float]]:
        image = self.range_image
        if not image:
            return None
        return image[:self.horizontal_resolution * self.number_of_layers]

    def getRangeImageArray(self) -> Optional[List[List[float]]]:
        array = [self.getLayerRangeImage(i) for i in range(self.number_of_layers)]
        return None if any(layer is None for layer in array) else array

    def getLayerRangeImage(self, layer: int) -> Optional[List[float]]:
        image = wb.wb_lidar_get_layer_range_image(self._tag, layer)
        if not image:
            return None
        return image[:self.horizontal_resolution]

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

    def getPointCloud(self, data_type: str = 'list') -> Optional[Union[List[LidarPoint], bytes]]:
        if not Lidar._check_data_type('getPointCloud', data_type):
            return None
        points = wb.wb_lidar_get_point_cloud(self._tag)
        if not points:  # NULL, e.g. before the first step or for a disabled point cloud
            return None
        return Lidar._unpack_points(points, self.number_of_points, data_type)

    def getLayerPointCloud(self, layer: int, data_type: str = 'list') -> Optional[Union[List[LidarPoint], bytes]]:
        if not Lidar._check_data_type('getLayerPointCloud', data_type):
            return None
        points = wb.wb_lidar_get_layer_point_cloud(self._tag, layer)
        if not points:
            return None
        return Lidar._unpack_points(points, self.number_of_points // self.number_of_layers, data_type)

    @staticmethod
    def _check_data_type(name: str, data_type: str) -> bool:
        if data_type in ('list', 'buffer'):
            return True
        print(f"Error: {name}(): 'data_type' cannot be '{data_type}', supported values are 'list' and 'buffer'.",
              file=sys.stderr)
        return False

    @staticmethod
    def _unpack_points(points, number_of_points: int, data_type: str) -> Union[List[LidarPoint], bytes]:
        data = ctypes.string_at(points, number_of_points * 20)
        if data_type == 'buffer':
            return data
        return [LidarPoint(data, i) for i in range(number_of_points)]
