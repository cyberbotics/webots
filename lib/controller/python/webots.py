# Copyright 1996-2021 Cyberbotics Ltd.
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
import os
import sys
import typing

_wb = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'Controller.dll'))


def _constant(name):
    return ctypes.c_int.in_dll(_wb, 'wb_' + name).value


_wb.wb_robot_get_basic_time_step.restype = ctypes.c_double


class Robot:
    created = None

    def __init__(self):
        if Robot.created:
            print('Error: only one Robot instance can be created per controller process.', file=sys.stderr)
            return
        Robot.created = self
        _wb.wb_robot_init()

    def step(self, time_step: int) -> int:
        return _wb.wb_robot_step(time_step)


_wb.wb_keyboard_get_key.restype = ctypes.c_int
_wb.wb_keyboard_get_sampling_period.restype = ctypes.c_int
_wb.wb_keyboard_enable.argtypes = [ctypes.c_int]


class Keyboard:
    END = _constant('KEYBOARD_END')
    HOME = _constant('KEYBOARD_HOME')
    LEFT = _constant('KEYBOARD_LEFT')
    UP = _constant('KEYBOARD_UP')
    RIGHT = _constant('KEYBOARD_RIGHT')
    DOWN = _constant('KEYBOARD_DOWN')
    PAGEUP = _constant('KEYBOARD_PAGEUP')
    PAGEDOWN = _constant('KEYBOARD_PAGEDOWN')
    NUMPAD_HOME = _constant('KEYBOARD_NUMPAD_HOME')
    NUMPAD_LEFT = _constant('KEYBOARD_NUMPAD_LEFT')
    NUMPAD_UP = _constant('KEYBOARD_NUMPAD_UP')
    NUMPAD_RIGHT = _constant('KEYBOARD_NUMPAD_RIGHT')
    NUMPAD_DOWN = _constant('KEYBOARD_NUMPAD_DOWN')
    NUMPAD_END = _constant('KEYBOARD_NUMPAD_END')
    KEY = _constant('KEYBOARD_KEY')
    SHIFT = _constant('KEYBOARD_SHIFT')
    CONTROL = _constant('KEYBOARD_CONTROL')
    ALT = _constant('KEYBOARD_ALT')

    def __init__(self, samplingPeriod: int = None):
        self.samplingPeriod = int(_wb.wb_robot_get_basic_time_step()) if samplingPeriod is None else samplingPeriod

    @property
    def samplingPeriod(self) -> int:
        return _wb.wb_keyboard_get_sampling_period()

    @samplingPeriod.setter
    def samplingPeriod(self, p: int):
        _wb.wb_keyboard_enable(p)

    def getKeyCode(self) -> int:
        return _wb.wb_keyboard_get_key()

    def getKey(self) -> str:
        k = _wb.wb_keyboard_get_key()
        s = ''
        if k & Keyboard.SHIFT != 0:
            s += 'shift-'
        if k & Keyboard.CONTROL != 0:
            s += 'control-'
        if k & Keyboard.ALT != 0:
            s += 'atl-'
        k &= Keyboard.KEY
        if k == Keyboard.END:
            s += 'end'
        elif k == Keyboard.HOME:
            s += 'home'
        elif k == Keyboard.LEFT:
            s += 'left'
        elif k == Keyboard.RIGHT:
            s += 'right'
        elif k == Keyboard.UP:
            s += 'up'
        elif k == Keyboard.DOWN:
            s += 'down'
        elif k == Keyboard.PAGEUP:
            s += 'page up'
        elif k == Keyboard.PAGEDOWN:
            s += 'page down'
        elif k == Keyboard.NUMPAD_END:
            s += 'numpad end'
        elif k == Keyboard.NUMPAD_HOME:
            s += 'numpad home'
        elif k == Keyboard.NUMPAD_LEFT:
            s += 'numpad left'
        elif k == Keyboard.NUMPAD_RIGHT:
            s += 'numpad right'
        elif k == Keyboard.NUMPAD_UP:
            s += 'numpad up'
        elif k == Keyboard.NUMPAD_DOWN:
            s += 'numpad down'
        else:
            s += chr(k)
        return s


class Supervisor(Robot):
    def __init__(self):
        super().__init__()

    def simulationQuit(status: int) -> None:
        _wb.wb_supervisor_simulation_quit(status)


_wb.wb_supervisor_node_get_from_def.argtypes = [ctypes.c_char_p]
_wb.wb_supervisor_node_get_from_def.restype = ctypes.c_void_p
_wb.wb_supervisor_node_get_type_name.restype = ctypes.c_char_p


class Node:
    def __init__(self, DEF=None):
        if DEF:
            self._ref = _wb.wb_supervisor_node_get_from_def(str.encode(DEF))

    @property
    def typeName(self) -> str:
        return _wb.wb_supervisor_node_get_type_name(self._ref)


_wb.wb_supervisor_node_get_field.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
_wb.wb_supervisor_node_get_field.restype = ctypes.c_void_p
_wb.wb_supervisor_field_get_type.argtypes = [ctypes.c_void_p]
_wb.wb_supervisor_field_get_type.restypes = ctypes.c_int
_wb.wb_supervisor_field_get_sf_bool.restype = ctypes.c_int
_wb.wb_supervisor_field_get_sf_int32.restype = ctypes.c_int
_wb.wb_supervisor_field_get_sf_float.restype = ctypes.c_double
_wb.wb_supervisor_field_get_sf_vec2f.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_sf_vec3f.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_sf_vec3f.argtypes = [ctypes.c_void_p]
_wb.wb_supervisor_field_get_sf_rotation.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_sf_color.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_sf_string.restype = ctypes.c_char_p
_wb.wb_supervisor_field_get_sf_node.restype = ctypes.c_int
_wb.wb_supervisor_field_get_mf_bool.restype = ctypes.c_int
_wb.wb_supervisor_field_get_mf_int32.restype = ctypes.c_int
_wb.wb_supervisor_field_get_mf_float.restype = ctypes.c_double
_wb.wb_supervisor_field_get_mf_vec2f.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_mf_vec3f.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_mf_rotation.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_mf_color.restype = ctypes.POINTER(ctypes.c_double)
_wb.wb_supervisor_field_get_mf_string.restype = ctypes.c_char_p
_wb.wb_supervisor_field_get_mf_node.restype = ctypes.c_int

_wb.wb_supervisor_field_set_sf_vec3f.argtypes = [ctypes.c_void_p, ctypes.c_double * 3]


class Field:
    SF_BOOL = _constant('SF_BOOL')
    SF_INT32 = _constant('SF_INT32')
    SF_FLOAT = _constant('SF_FLOAT')
    SF_VEC2F = _constant('SF_VEC2F')
    SF_VEC3F = _constant('SF_VEC3F')
    SF_ROTATION = _constant('SF_ROTATION')
    SF_COLOR = _constant('SF_COLOR')
    SF_STRING = _constant('SF_STRING')
    SF_NODE = _constant('SF_NODE')
    MF_BOOL = _constant('MF_BOOL')
    MF_INT32 = _constant('MF_INT32')
    MF_FLOAT = _constant('MF_FLOAT')
    MF_VEC2F = _constant('MF_VEC2F')
    MF_VEC3F = _constant('MF_VEC3F')
    MF_ROTATION = _constant('MF_ROTATION')
    MF_COLOR = _constant('MF_COLOR')
    MF_STRING = _constant('MF_STRING')
    MF_NODE = _constant('MF_NODE')

    def __init__(self, node, name):
        self._ref = _wb.wb_supervisor_node_get_field(node._ref, str.encode(name))
        self.type = _wb.wb_supervisor_field_get_type(self._ref)

    @property
    def value(self) -> typing.Union[bool, int, float, str,
                                    typing.List[bool], typing.List[int], typing.List[float], typing.List[str]]:
        if self.type == Field.SF_BOOL:
            return _wb.wb_supervisor_field_get_sf_bool(self._ref)
        if self.type == Field.SF_INT32:
            return _wb.wb_supervisor_field_get_sf_int32(self._ref)
        if self.type == Field.SF_FLOAT:
            return _wb.wb_supervisor_field_get_sf_float(self._ref)
        if self.type == Field.SF_VEC2F:
            return _wb.wb_supervisor_field_get_sf_vec2f(self._ref)[:2]
        if self.type == Field.SF_VEC3F:
            return _wb.wb_supervisor_field_get_sf_vec3f(self._ref)[:3]
        if self.type == Field.SF_ROTATION:
            return _wb.wb_supervisor_field_get_sf_rotation(self._ref)[:4]
        if self.type == Field.SF_COLOR:
            return _wb.wb_supervisor_field_get_sf_color(self._ref)[:3]
        return None

    @value.setter
    def value(self, p: typing.Union[bool, int, float, str,
                                    typing.List[bool], typing.List[int], typing.List[float], typing.List[str]]):
        if self.type == Field.SF_VEC3F:
            _wb.wb_supervisor_field_set_sf_vec3f(self._ref, (ctypes.c_double * 3)(*p))


_wb.wb_distance_sensor_get_value.restype = ctypes.c_double
_wb.wb_distance_sensor_get_sampling_period.restype = ctypes.c_int


class DistanceSensor:
    def __init__(self, name: str, samplingPeriod: int = None):
        self._ref = _wb.wb_robot_get_device(str.encode(name))
        self.samplingPeriod = int(_wb.wb_robot_get_basic_time_step()) if samplingPeriod is None else samplingPeriod

    @property
    def samplingPeriod(self) -> int:
        return _wb.wb_distance_sensor_get_sampling_period(self._ref)

    @samplingPeriod.setter
    def samplingPeriod(self, p: int):
        _wb.wb_distance_sensor_enable(self._ref, p)

    @property
    def value(self) -> float:
        return _wb.wb_distance_sensor_get_value(self._ref)


_wb.wb_emitter_send.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int]


class Emitter:
    def __init__(self, name: str):
        self._ref = _wb.wb_robot_get_device(str.encode(name))

    def send(self, message: typing.Union[str, bytes], length: int = None):
        if isinstance(message, str):
            _wb.wb_emitter_send(self._ref, str.encode(message), len(message))
        elif isinstance(message, bytes):
            if length is None:
                print('Emitter.send(): missing byte buffer length', file=sys.stderr)
            else:
                _wb.wb_emitter_send(self._ref, message, length)
        else:
            print('Emitter.send(): unsupported data type', file=sys.stderr)


_wb.wb_motor_get_target_position.restype = ctypes.c_double
_wb.wb_motor_get_velocity.restype = ctypes.c_double


class Motor:
    ROTATIONAL = _constant('ROTATIONAL')
    LINEAR = _constant('LINEAR')

    def __init__(self, name: str):
        self._ref = _wb.wb_robot_get_device(str.encode(name))

    @property
    def position(self) -> float:
        return _wb.wb_motor_get_target_position(self._ref)

    @position.setter
    def position(self, p: float):
        _wb.wb_motor_set_position(self._ref, ctypes.c_double(p))

    @property
    def velocity(self) -> float:
        return _wb.wb_motor_get_velocity(self._ref)

    @velocity.setter
    def velocity(self, v: float):
        _wb.wb_motor_set_velocity(self._ref, ctypes.c_double(v))

    @property
    def type(self) -> int:
        return _wb.wb_motor_get_type(self._ref)
