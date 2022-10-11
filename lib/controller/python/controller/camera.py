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
from typing import Union


class Camera(Sensor):
    wb.wb_camera_get_image.restype = ctypes.POINTER(ctypes.c_ubyte)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_camera_enable
        self._get_sampling_period = wb.wb_camera_get_sampling_period
        super().__init__(name, sampling_period)

    def getExposure(self) -> float:
        return self.exposure

    def getFocalDistance(self) -> float:
        return self.focal_distance

    def getFocalLength(self) -> float:
        return self.focal_length

    def getFov(self) -> float:
        return self.fov

    def setFocalDistance(self, d: float):
        self.focal_distance = d

    def setFov(self, f: float):
        self.fov = f

    def getHeight(self):
        return self.height

    def getImage(self) -> bytes:
        return wb.wb_camera_get_image(self._tag)

    @staticmethod
    def imageGetRed(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x) + 2]

    @staticmethod
    def imageGetGreen(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x) + 1]

    @staticmethod
    def imageGetBlue(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x)]

    def getMaxFocalDistance(self) -> float:
        return self.max_focal_distance

    def getMaxFov(self) -> float:
        return self.maxFov

    def getMinFocalDistance(self) -> float:
        return self.min_focal_distance

    def getMinFov(self) -> float:
        return self.minFov

    def getNear(self) -> float:
        return self.near

    def getWidth(self):
        return self.width

    def saveImage(self, filename: str, quality: int) -> int:
        return wb.wb_camera_save_image(self._tag, str.encode(filename), quality)

    @property
    def exposure(self) -> float:
        return wb.wb_camera_get_exposure(self._tag)

    @exposure.setter
    def exposure(self, e: float):
        wb.wb_camera_set_exposure(self._tag, e)

    @property
    def focal_distance(self) -> float:
        return wb.wb_camera_get_focal_distance(self._tag)

    @focal_distance.setter
    def focal_distance(self, d: float):
        wb.wb_camera_set_focal_distance(self._tag, d)

    @property
    def focal_length(self) -> float:
        return wb.wb_camera_get_focal_length(self._tag)

    @property
    def max_focal_distance(self) -> float:
        return wb.wb_camera_get_max_focal_distance(self._tag)

    @property
    def min_focal_distance(self) -> float:
        return wb.wb_camera_get_min_focal_distance(self._tag)

    @property
    def fov(self) -> float:
        return wb.wb_camera_get_fov(self._tag)

    @fov.setter
    def fov(self, f: float):
        wb.wb_camera_set_fov(self._tag, f)

    @property
    def height(self) -> int:
        return wb.wb_camera_get_height(self._tag)

    @property
    def minFov(self) -> float:
        return wb.wb_camera_get_min_fov(self._tag)

    @property
    def maxFov(self) -> float:
        return wb.wb_camera_get_max_fov(self._tag)

    @property
    def near(self) -> float:
        return wb.wb_camera_get_near(self._tag)

    @property
    def width(self) -> int:
        return wb.wb_camera_get_width(self._tag)
