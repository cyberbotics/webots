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

_wb = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'Controller.dll'))


class Robot:
    created = False

    def __init__(self):
        if Robot.created:
            print('Error: only one Robot instance can be created per controller process.')
            return
        Robot.created = True
        _wb.wb_robot_init()

    def step(self, time_step: int) -> int:
        return _wb.wb_robot_step(time_step)


_wb.wb_distance_sensor_get_value.restype = ctypes.c_double


class DistanceSensor:
    def __init__(self, name: str):
        self.id = _wb.wb_robot_get_device(str.encode(name))

    def enable(self, time_step: int):
        _wb.wb_distance_sensor_enable(self.id, time_step)

    @property
    def value(self) -> float:
        return _wb.wb_distance_sensor_get_value(self.id)


_wb.wb_motor_get_target_position.restype = ctypes.c_double
_wb.wb_motor_get_velocity.restype = ctypes.c_double


class Motor:
    ROTATIONAL = ctypes.c_int.in_dll(_wb, 'wb_ROTATIONAL').value
    LINEAR = ctypes.c_int.in_dll(_wb, 'wb_LINEAR').value

    def __init__(self, name: str):
        self.id = _wb.wb_robot_get_device(str.encode(name))

    @property
    def position(self) -> float:
        return _wb.wb_motor_get_target_position(self.id)

    @position.setter
    def position(self, p: float):
        _wb.wb_motor_set_position(self.id, ctypes.c_double(p))

    @property
    def velocity(self) -> float:
        return _wb.wb_motor_get_velocity(self.id)

    @velocity.setter
    def velocity(self, v: float):
        _wb.wb_motor_set_velocity(self.id, ctypes.c_double(v))

    @property
    def type(self) -> int:
        return _wb.wb_motor_get_type(self.id)
