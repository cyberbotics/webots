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
from .wb import wb
from typing import List, Union


class CameraRecognitionObject(ctypes.Structure):
    _fields_ = [('id', ctypes.c_int),
                ('position', ctypes.c_double * 3),
                ('orientation', ctypes.c_double * 4),
                ('size', ctypes.c_double * 2),
                ('position_on_image', ctypes.c_int * 2),
                ('size_on_image', ctypes.c_int * 2),
                ('number_of_colors', ctypes.c_int),
                ('colors', ctypes.POINTER(ctypes.c_double)),
                ('_model', ctypes.c_char_p)]

    def getId(self) -> int:
        return self.id

    def getPosition(self) -> List[float]:
        return self.position

    def getOrientation(self) -> List[float]:
        return self.orientation

    def getSize(self) -> List[float]:
        return self.size

    def getPositionOnImage(self) -> List[int]:
        return self.position_on_image

    def getSizeOnImage(self) -> List[int]:
        return self.size_on_image

    def getNumberOfColors(self) -> int:
        return self.number_of_colors

    def getColors(self) -> List[float]:
        return self.colors

    def getModel(self) -> str:
        return self.model

    @property
    def model(self) -> str:
        return self._model.decode()


class Camera(Sensor):
    wb.wb_camera_get_fov.restype = ctypes.c_double
    wb.wb_camera_get_exposure.restype = ctypes.c_double
    wb.wb_camera_get_focal_distance.restype = ctypes.c_double
    wb.wb_camera_get_focal_length.restype = ctypes.c_double
    wb.wb_camera_get_max_fov.restype = ctypes.c_double
    wb.wb_camera_get_min_fov.restype = ctypes.c_double
    wb.wb_camera_get_max_focal_distance.restype = ctypes.c_double
    wb.wb_camera_get_min_focal_distance.restype = ctypes.c_double
    wb.wb_camera_get_near.restype = ctypes.c_double

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

    def getHeight(self) -> int:
        return self.height

    def getImage(self) -> bytes:
        return self.image

    def getImageArray(self) -> List[List[List[int]]]:
        array = []
        image = self.image
        if not image:
            return None
        i = 0
        for x in range(self.width):
            line = []
            for y in range(self.height):
                line.append([image[i + 2], image[i + 1], image[i]])  # RGB pixel
                i += 4
            array.append(line)
        return array

    @staticmethod
    def imageGetRed(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x) + 2]

    @staticmethod
    def imageGetGreen(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x) + 1]

    @staticmethod
    def imageGetBlue(image: bytes, width: int, x: int, y: int) -> int:
        return image[4 * (y * width + x)]

    @staticmethod
    def imageGetGray(image: bytes, width: int, x: int, y: int) -> int:
        return (image[4 * (y * width + x) + 2] + image[4 * (y * width + x) + 1] + image[4 * (y * width + x)]) / 3

    def getMaxFocalDistance(self) -> float:
        return self.max_focal_distance

    def getMaxFov(self) -> float:
        return self.max_fov

    def getMinFocalDistance(self) -> float:
        return self.min_focal_distance

    def getMinFov(self) -> float:
        return self.min_fov

    def getNear(self) -> float:
        return self.near

    def getWidth(self) -> int:
        return self.width

    def saveImage(self, filename: str, quality: int) -> int:
        return wb.wb_camera_save_image(self._tag, str.encode(filename), quality)

    def setExposure(self, exposure: float):
        self.exposure = exposure

    def setFocalDistance(self, d: float):
        self.focal_distance = d

    def setFov(self, f: float):
        self.fov = f

    @property
    def image(self) -> bytes:
        wb.wb_camera_get_image.restype = ctypes.POINTER(ctypes.c_ubyte * (4 * self.width * self.height))
        return bytes(wb.wb_camera_get_image(self._tag).contents)

    @property
    def segmentation_image(self) -> bytes:
        wb.wb_camera_recognition_get_segmentation_image.restype = ctypes.POINTER(
            ctypes.c_ubyte * (4 * self.width * self.height)
        )
        return bytes(wb.wb_camera_recognition_get_segmentation_image(self._tag).contents)

    @property
    def exposure(self) -> float:
        return wb.wb_camera_get_exposure(self._tag)

    @exposure.setter
    def exposure(self, exposure: float):
        wb.wb_camera_set_exposure(self._tag, ctypes.c_double(exposure))

    @property
    def focal_distance(self) -> float:
        return wb.wb_camera_get_focal_distance(self._tag)

    @focal_distance.setter
    def focal_distance(self, focal_distance: float):
        wb.wb_camera_set_focal_distance(self._tag, ctypes.c_double(focal_distance))

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
    def fov(self, fov: float):
        wb.wb_camera_set_fov(self._tag, ctypes.c_double(fov))

    @property
    def height(self) -> int:
        return wb.wb_camera_get_height(self._tag)

    @property
    def min_fov(self) -> float:
        return wb.wb_camera_get_min_fov(self._tag)

    @property
    def max_fov(self) -> float:
        return wb.wb_camera_get_max_fov(self._tag)

    @property
    def near(self) -> float:
        return wb.wb_camera_get_near(self._tag)

    @property
    def width(self) -> int:
        return wb.wb_camera_get_width(self._tag)

    wb.wb_camera_recognition_get_objects.restype = ctypes.POINTER(CameraRecognitionObject)

    def getRecognitionNumberOfObjects(self) -> int:
        return wb.wb_camera_recognition_get_number_of_objects(self._tag)

    def getRecognitionObjects(self) -> List[CameraRecognitionObject]:
        return wb.wb_camera_recognition_get_objects(self._tag)[:wb.wb_camera_recognition_get_number_of_objects(self._tag)]

    def getRecognitionSamplingPeriod(self) -> int:
        return wb.wb_camera_recognition_get_sampling_period(self._tag)

    def hasRecognition(self) -> bool:
        return wb.wb_camera_has_recognition(self._tag) != 0

    def recognitionDisable(self):
        wb.wb_camera_recognition_disable(self._tag)

    def recognitionEnable(self, sampling_period: int):
        wb.wb_camera_recognition_enable(self._tag, sampling_period)

    def disableRecognitionSegmentation(self):
        wb.wb_camera_recognition_disable_segmentation(self._tag)

    def enableRecognitionSegmentation(self):
        wb.wb_camera_recognition_enable_segmentation(self._tag)

    def hasRecognitionSegmentation(self) -> bool:
        return wb.wb_camera_recognition_has_segmentation(self._tag) != 0

    def isRecognitionSegmentationEnabled(self) -> bool:
        return wb.wb_camera_recognition_is_segmentation_enabled(self._tag) != 0

    def getRecognitionSegmentationImage(self) -> bytes:
        return self.segmentation_image

    def getRecognitionSegmentationImageArray(self) -> List[List[List[int]]]:
        array = []
        image = self.getRecognitionSegmentationImage()
        if not image:
            return None
        i = 0
        for x in range(self.width):
            line = []
            for y in range(self.height):
                line.append([image[i + 2], image[i + 1], image[i]])  # RGB pixel
                i += 4
            array.append(line)
        return array

    def saveRecognitionSegmentationImage(self, filename: str, quality: int) -> int:
        return wb.wb_camera_recognition_save_segmentation_image(self._tag, str.encode(filename), quality)
