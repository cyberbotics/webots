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
from .camera import Camera
from .device import Device
from typing import Union, List


class Display(Device):
    RGB = 3
    RGBA = 4
    ARGB = 5
    BGRA = 6
    ABGR = 7
    wb.wb_display_image_copy.restype = ctypes.c_void_p
    wb.wb_display_image_load.restype = ctypes.c_void_p
    wb.wb_display_image_new.restype = ctypes.c_void_p
    wb.wb_display_image_new.argtypes = [ctypes.c_void_p,
                                        ctypes.c_int,
                                        ctypes.c_int,
                                        ctypes.c_void_p,
                                        ctypes.c_int]

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def attachCamera(self, camera: Camera):
        wb.wb_display_attach_camera(self._tag, camera._tag)

    def detachCamera(self):
        wb.wb_display_detach_camera(self._tag)

    def drawLine(self, x1: int, y1: int, x2: int, y2: int):
        wb.wb_display_draw_line(self._tag, int(x1), int(y1), int(x2), int(y2))

    def drawOval(self, cx: int, cy: int, a: int, b: int):
        wb.wb_display_draw_oval(self._tag, int(cx), int(cy), int(a), int(b))

    def drawPixel(self, x: int, y: int):
        wb.wb_display_draw_pixel(self._tag, int(x), int(y))

    def drawPolygon(self, x: List[int], y: List[int]):
        wb.wb_display_draw_polygon(self._tag,
                                   (ctypes.c_int * len(x))(*x),
                                   (ctypes.c_int * len(y))(*y),
                                   min(len(x), len(y)))

    def drawRectangle(self, x: int, y: int, width: int, height: int):
        wb.wb_display_draw_rectangle(self._tag, int(x), int(y), int(width), int(height))

    def drawText(self, text: str, x: int, y: int):
        wb.wb_display_draw_text(self._tag, str.encode(text), int(x), int(y))

    def fillOval(self, cx: int, cy: int, a: int, b: int):
        wb.wb_display_fill_oval(self._tag, int(cx), int(cy), int(a), int(b))

    def fillPolygon(self, x: List[int], y: List[int]):
        wb.wb_display_fill_polygon(self._tag,
                                   (ctypes.c_int * len(x))(*x),
                                   (ctypes.c_int * len(y))(*y),
                                   min(len(x), len(y)))

    def fillRectangle(self, x: int, y: int, width: int, height: int):
        wb.wb_display_fill_rectangle(self._tag, int(x), int(y), int(width), int(height))

    def getHeight(self) -> int:
        return self.height

    def getWidth(self) -> int:
        return self.width

    def imageCopy(self, x: int, y: int, width: int, height: int) -> int:
        return wb.wb_display_image_copy(self._tag, int(x), int(y), int(width), int(height))

    def imageDelete(self, image):
        wb.wb_display_image_delete(self._tag, ctypes.c_void_p(image))

    def imageNew(self, data: bytes, format: int, width: int, height: int) -> int:
        return wb.wb_display_image_new(self._tag, int(width), int(height), data, format)

    def imageLoad(self, filename: str) -> int:
        return wb.wb_display_image_load(self._tag, str.encode(filename))

    def imagePaste(self, image, x: int, y: int, blend: bool):
        wb.wb_display_image_paste(self._tag, ctypes.c_void_p(image), int(x), int(y), ctypes.c_int(blend))

    def imageSave(self, image, filename: str):
        wb.wb_display_image_save(self._tag, ctypes.c_void_p(image), str.encode(filename))

    def setAlpha(self, alpha: float):
        wb.wb_display_set_alpha(self._tag, ctypes.c_double(alpha))

    alpha = property(fset=setAlpha)

    def setColor(self, color: int):
        wb.wb_display_set_color(self._tag, color)

    color = property(fset=setColor)

    def setFont(self, font: str, size: int, anti_aliasing: bool):
        wb.wb_display_set_font(self._tag, str.encode(font), size, ctypes.c_int(anti_aliasing))

    def setOpacity(self, opacity: float):
        wb.wb_display_set_opacity(self._tag, ctypes.c_double(opacity))

    opacity = property(fset=setOpacity)

    @property
    def width(self) -> int:
        return wb.wb_display_get_width(self._tag)

    @property
    def height(self) -> int:
        return wb.wb_display_get_height(self._tag)
