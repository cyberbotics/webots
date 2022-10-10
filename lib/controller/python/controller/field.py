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
import typing
from controller.wb import wb
from controller.constants import constant


class Field:
    SF_BOOL = constant('SF_BOOL')
    SF_INT32 = constant('SF_INT32')
    SF_FLOAT = constant('SF_FLOAT')
    SF_VEC2F = constant('SF_VEC2F')
    SF_VEC3F = constant('SF_VEC3F')
    SF_ROTATION = constant('SF_ROTATION')
    SF_COLOR = constant('SF_COLOR')
    SF_STRING = constant('SF_STRING')
    SF_NODE = constant('SF_NODE')
    MF_BOOL = constant('MF_BOOL')
    MF_INT32 = constant('MF_INT32')
    MF_FLOAT = constant('MF_FLOAT')
    MF_VEC2F = constant('MF_VEC2F')
    MF_VEC3F = constant('MF_VEC3F')
    MF_ROTATION = constant('MF_ROTATION')
    MF_COLOR = constant('MF_COLOR')
    MF_STRING = constant('MF_STRING')
    MF_NODE = constant('MF_NODE')

    wb.wb_supervisor_node_get_field.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    wb.wb_supervisor_node_get_field.restype = ctypes.c_void_p
    wb.wb_supervisor_field_get_type.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_bool.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_int32.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_float.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_float.restype = ctypes.c_double
    wb.wb_supervisor_field_get_sf_vec2f.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_vec2f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_vec3f.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_vec3f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_rotation.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_rotation.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_color.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_color.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_string.argtypes = [ctypes.c_void_p]
    wb.wb_supervisor_field_get_sf_string.restype = ctypes.c_char_p

    def __init__(self, node, name):
        self._ref = wb.wb_supervisor_node_get_field(node._ref, str.encode(name))
        self.type = wb.wb_supervisor_field_get_type(self._ref)

    @property
    def value(self) -> typing.Union[bool, int, float, str,
                                    typing.List[bool], typing.List[int], typing.List[float], typing.List[str]]:
        if self.type == Field.SF_BOOL:
            return wb.wb_supervisor_field_get_sf_bool(self._ref)
        if self.type == Field.SF_INT32:
            return wb.wb_supervisor_field_get_sf_int32(self._ref)
        if self.type == Field.SF_FLOAT:
            return wb.wb_supervisor_field_get_sf_float(self._ref)
        if self.type == Field.SF_VEC2F:
            return wb.wb_supervisor_field_get_sf_vec2f(self._ref)[:2]
        if self.type == Field.SF_VEC3F:
            return wb.wb_supervisor_field_get_sf_vec3f(self._ref)[:3]
        if self.type == Field.SF_ROTATION:
            return wb.wb_supervisor_field_get_sf_rotation(self._ref)[:4]
        if self.type == Field.SF_COLOR:
            return wb.wb_supervisor_field_get_sf_color(self._ref)[:3]
        return None

    wb.wb_supervisor_field_set_sf_vec2f.argtypes = [ctypes.c_void_p, ctypes.c_double * 2]
    wb.wb_supervisor_field_set_sf_vec3f.argtypes = [ctypes.c_void_p, ctypes.c_double * 3]
    wb.wb_supervisor_field_set_sf_rotation.argtypes = [ctypes.c_void_p, ctypes.c_double * 4]
    wb.wb_supervisor_field_set_sf_color.argtypes = [ctypes.c_void_p, ctypes.c_double * 3]

    @value.setter
    def value(self, p: typing.Union[bool, int, float, str,
                                    typing.List[bool], typing.List[int], typing.List[float], typing.List[str]]):
        if self.type == Field.SF_VEC2F:
            wb.wb_supervisor_field_set_sf_vec2f(self._ref, (ctypes.c_double * 2)(*p))
        elif self.type == Field.SF_VEC3F:
            wb.wb_supervisor_field_set_sf_vec3f(self._ref, (ctypes.c_double * 3)(*p))
        elif self.type == Field.SF_ROTATION:
            wb.wb_supervisor_field_set_sf_rotation(self._ref, (ctypes.c_double * 4)(*p))
        elif self.type == Field.SF_COLOR:
            wb.wb_supervisor_field_set_sf_color(self._ref, (ctypes.c_double * 3)(*p))
