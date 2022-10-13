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
from controller.sensor import Sensor
from controller.wb import wb
from typing import List, Union


class Lidar(Sensor):
    wb.wb_lidar_get_fov.restype = ctypes.c_double
    wb.wb_lidar_get_vertical_fov.restype = ctypes.c_double
    wb.wb_lidar_get_max_frequency.restype = ctypes.c_double
    wb.wb_lidar_get_min_frequency.restype = ctypes.c_double
    wb.wb_lidar_get_max_range.restype = ctypes.c_double
    wb.wb_lidar_get_min_range.restype = ctypes.c_double
    wb.wb_lidar_get_range_image.restype = ctypes.POINTER(ctypes.c_float)
    wb.wb_lidar_get_layer_range_image.restype = ctypes.POINTER(ctypes.c_float)
    wb.wb_lidar_get_point_cloud.restype = ctypes.POINTER(ctypes.c_ubyte)

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
        return wb.wb_lidar_get_range_image(self._tag)

    def getLayerRangeImage(self) -> List[List[float]]:
        return wb.wb_lidar_get_layer_range_image(self._tag)

    def getImageArray(self) -> List[int]:
        return [x for x in self.getImage()]

    def saveImage(self, filename: str, quality: int) -> int:
        return wb.wb_lidar_save_image(self._tag, str.encode(filename), quality)

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

    def getPointCloudImage(self) -> bytes:
        return wb.wb_lidar_recognition_get_segmentation_image(self._tag)

    def getPointCloudImageArray(self) -> List[int]:
        return [x for x in self.getPointCloudImage()]

    def savePointCloudImage(self, filename: str, quality: int) -> int:
        return wb.wb_lidar_recognition_save_segmentation_image(self._tag, str.encode(filename), quality)
