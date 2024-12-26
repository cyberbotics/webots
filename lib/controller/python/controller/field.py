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
from .constants import constant
import sys
import typing


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

    wb.wb_supervisor_field_get_name.restype = ctypes.c_char_p
    wb.wb_supervisor_field_get_type_name.restype = ctypes.c_char_p
    wb.wb_supervisor_field_get_actual_field.restype = ctypes.c_void_p
    wb.wb_supervisor_field_get_sf_float.restype = ctypes.c_double
    wb.wb_supervisor_field_get_sf_vec2f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_vec3f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_rotation.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_color.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_sf_string.restype = ctypes.c_char_p
    wb.wb_supervisor_field_get_mf_float.restype = ctypes.c_double
    wb.wb_supervisor_field_get_mf_vec2f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_mf_vec3f.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_mf_rotation.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_mf_color.restype = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_field_get_mf_string.restype = ctypes.c_char_p
    wb.wb_supervisor_field_set_sf_vec2f.argtypes = [ctypes.c_void_p, ctypes.c_double * 2]
    wb.wb_supervisor_field_set_sf_vec3f.argtypes = [ctypes.c_void_p, ctypes.c_double * 3]
    wb.wb_supervisor_field_set_sf_rotation.argtypes = [ctypes.c_void_p, ctypes.c_double * 4]
    wb.wb_supervisor_field_set_sf_color.argtypes = [ctypes.c_void_p, ctypes.c_double * 3]
    wb.wb_supervisor_field_get_mf_float.restype = ctypes.c_double
    wb.wb_supervisor_virtual_reality_headset_get_position = ctypes.POINTER(ctypes.c_double)
    wb.wb_supervisor_virtual_reality_headset_get_orientation = ctypes.POINTER(ctypes.c_double)

    def __init__(self, ref: ctypes.c_void_p):
        self._ref = ctypes.c_void_p(ref)
        if self._ref:
            self.type = wb.wb_supervisor_field_get_type(self._ref)
        else:
            self.type = None

    def getName(self) -> str:
        return self.name

    def getType(self) -> int:
        return self.type

    def getTypeName(self) -> str:
        return self.type_name

    def getCount(self) -> int:
        return self.count

    def getActualField(self) -> 'Field':
        field = wb.wb_supervisor_field_get_actual_field(self._ref)
        return Field(field) if field else None

    def enableSFTracking(self, samplingPeriod: int):
        wb.wb_supervisor_field_enable_sf_tracking(self._ref, samplingPeriod)

    def disableSFTracking(self):
        wb.wb_supervisor_field_disable_sf_tracking(self._ref)

    def getSFBool(self) -> bool:
        return self.value

    def getSFInt32(self) -> int:
        return self.value

    def getSFFloat(self) -> float:
        return self.value

    def getSFVec2f(self) -> typing.List[float]:
        return self.value

    def getSFVec3f(self) -> typing.List[float]:
        return self.value

    def getSFRotation(self) -> typing.List[float]:
        return self.value

    def getSFColor(self) -> typing.List[float]:
        return self.value

    def getSFString(self) -> str:
        return self.value

    def getMFBool(self, index: int) -> bool:
        return wb.wb_supervisor_field_get_mf_bool(self._ref, index) != 0

    def getMFInt32(self, index: int) -> int:
        return wb.wb_supervisor_field_get_mf_int32(self._ref, index)

    def getMFFloat(self, index: int) -> float:
        return wb.wb_supervisor_field_get_mf_float(self._ref, index)

    def getMFString(self, index: int) -> str:
        return wb.wb_supervisor_field_get_mf_string(self._ref, index).decode()

    def getMFVec2f(self, index: int) -> typing.List[float]:
        return wb.wb_supervisor_field_get_mf_vec2f(self._ref, index)[:2]

    def getMFVec3f(self, index: int) -> typing.List[float]:
        return wb.wb_supervisor_field_get_mf_vec3f(self._ref, index)[:3]

    def getMFRotation(self, index: int) -> typing.List[float]:
        return wb.wb_supervisor_field_get_mf_rotation(self._ref, index)[:4]

    def getMFColor(self, index: int) -> typing.List[float]:
        return wb.wb_supervisor_field_get_mf_color(self._ref, index)[:3]

    def setSFBool(self, value: bool):
        self.value = value

    def setSFInt32(self, value: int):
        self.value = value

    def setSFFloat(self, value: float):
        self.value = value

    def setSFVec2f(self, value: typing.List[float]):
        if len(value) == 2:
            self.value = value
        else:
            print("Error: setSFVec2f(): length of 'value' argument should be 2.", file=sys.stderr)

    def setSFVec3f(self, value: typing.List[float]):
        if len(value) == 3:
            self.value = value
        else:
            print("Error: setSFVec3f(): length of 'value' argument should be 3.", file=sys.stderr)

    def setSFRotation(self, value: typing.List[float]):
        if len(value) == 4:
            self.value = value
        else:
            print("Error: setSFRotation(): length of 'value' argument should be 4.", file=sys.stderr)

    def setSFColor(self, value: typing.List[float]):
        if len(value) == 3:
            self.value = value
        else:
            print("Error: setSFColor(): length of 'value' argument should be 3.", file=sys.stderr)

    def setSFString(self, value: str):
        self.value = value

    def setMFBool(self, index, value: bool):
        wb.wb_supervisor_field_set_mf_bool(self._ref, index, 1 if value else 0)

    def setMFInt32(self, index, value: int):
        wb.wb_supervisor_field_set_mf_int32(self._ref, index, value)

    def setMFFloat(self, index, value: float):
        wb.wb_supervisor_field_set_mf_float(self._ref, index, ctypes.c_double(value))

    def setMFVec2f(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_set_mf_vec2f(self._ref, index, (ctypes.c_double * 2)(*value))

    def setMFVec3f(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_set_mf_vec3f(self._ref, index, (ctypes.c_double * 3)(*value))

    def setMFRotation(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_set_mf_rotation(self._ref, index, (ctypes.c_double * 4)(*value))

    def setMFColor(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_set_mf_color(self._ref, index, (ctypes.c_double * 3)(*value))

    def setMFString(self, index, value: str):
        wb.wb_supervisor_field_set_mf_string(self._ref, index, str.encode(value))

    def insertMFBool(self, index, value: bool):
        wb.wb_supervisor_field_insert_mf_bool(self._ref, index, 1 if value else 0)

    def insertMFInt32(self, index, value: int):
        wb.wb_supervisor_field_insert_mf_int32(self._ref, index, value)

    def insertMFFloat(self, index, value: float):
        wb.wb_supervisor_field_insert_mf_float(self._ref, index, ctypes.c_double(value))

    def insertMFVec2f(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_insert_mf_vec2f(self._ref, index, (ctypes.c_double * 2)(*value))

    def insertMFVec3f(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_insert_mf_vec3f(self._ref, index, (ctypes.c_double * 3)(*value))

    def insertMFRotation(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_insert_mf_rotation(self._ref, index, (ctypes.c_double * 4)(*value))

    def insertMFColor(self, index, value: typing.List[float]):
        wb.wb_supervisor_field_insert_mf_color(self._ref, index, (ctypes.c_double * 3)(*value))

    def insertMFString(self, index, value):
        wb.wb_supervisor_field_insert_mf_string(self._ref, index, str.encode(value))

    def removeMF(self, index):
        wb.wb_supervisor_field_remove_mf(self._ref, index)

    def removeSF(self):
        wb.wb_supervisor_field_remove_sf(self._ref)

    def importMFNodeFromString(self, position: int, nodeString: str):
        wb.wb_supervisor_field_import_mf_node_from_string(self._ref, position, str.encode(nodeString))

    def importSFNodeFromString(self, nodeString: str):
        wb.wb_supervisor_field_import_sf_node_from_string(self._ref, str.encode(nodeString))

    @property
    def name(self) -> str:
        return wb.wb_supervisor_field_get_name(self._ref).decode()

    @property
    def type_name(self) -> str:
        return wb.wb_supervisor_field_get_type_name(self._ref).decode()

    @property
    def count(self) -> int:
        return wb.wb_supervisor_field_get_count(self._ref)

    @property
    def value(self) -> typing.Union[bool, int, float, str, typing.List[float]]:
        if self.type == Field.SF_BOOL:
            return wb.wb_supervisor_field_get_sf_bool(self._ref)
        elif self.type == Field.SF_INT32:
            return wb.wb_supervisor_field_get_sf_int32(self._ref)
        elif self.type == Field.SF_FLOAT:
            return wb.wb_supervisor_field_get_sf_float(self._ref)
        elif self.type == Field.SF_STRING:
            return wb.wb_supervisor_field_get_sf_string(self._ref).decode()
        elif self.type == Field.SF_VEC2F:
            return wb.wb_supervisor_field_get_sf_vec2f(self._ref)[:2]
        elif self.type == Field.SF_VEC3F:
            return wb.wb_supervisor_field_get_sf_vec3f(self._ref)[:3]
        elif self.type == Field.SF_ROTATION:
            return wb.wb_supervisor_field_get_sf_rotation(self._ref)[:4]
        elif self.type == Field.SF_COLOR:
            return wb.wb_supervisor_field_get_sf_color(self._ref)[:3]
        elif self.type == Field.SF_NODE:
            return self.getSFNode()
        else:
            return None

    @value.setter
    def value(self, value: typing.Union[bool, int, float, str, typing.List[float]]):
        if self.type == Field.SF_BOOL and isinstance(value, bool):
            wb.wb_supervisor_field_set_sf_bool(self._ref, 1 if value else 0)
        elif self.type == Field.SF_INT32 and isinstance(value, int):
            wb.wb_supervisor_field_set_sf_int32(self._ref, value)
        elif self.type == Field.SF_FLOAT and isinstance(value, float):
            wb.wb_supervisor_field_set_sf_float(self._ref, ctypes.c_double(value))
        elif self.type == Field.SF_STRING and isinstance(value, str):
            wb.wb_supervisor_field_set_sf_string(self._ref, str.encode(value))
        elif self.type == Field.SF_VEC2F and isinstance(value, list) and len(value) == 2:
            wb.wb_supervisor_field_set_sf_vec2f(self._ref, (ctypes.c_double * 2)(*value))
        elif self.type == Field.SF_VEC3F and isinstance(value, list) and len(value) == 3:
            wb.wb_supervisor_field_set_sf_vec3f(self._ref, (ctypes.c_double * 3)(*value))
        elif self.type == Field.SF_ROTATION and isinstance(value, list) and len(value) == 4:
            wb.wb_supervisor_field_set_sf_rotation(self._ref, (ctypes.c_double * 4)(*value))
        elif self.type == Field.SF_COLOR and isinstance(value, list) and len(value) == 3:
            wb.wb_supervisor_field_set_sf_color(self._ref, (ctypes.c_double * 3)(*value))
        else:
            print("Error: new field value has wrong type or length.", file=sys.stderr)
