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


class Robot:
    controller = None

    def __init__(self):
        if Robot.controller is not None:
            print('Error: only one Robot instance can be created per controller process.')
            return
        self.WEBOTS_HOME = os.environ['WEBOTS_HOME']
        Robot.controller = ctypes.cdll.LoadLibrary(os.path.join(self.WEBOTS_HOME, 'lib', 'controller', 'Controller.dll'))
        Robot.controller.wb_robot_init()

    def step(self, time_step: int) -> int:
        return Robot.controller.wb_robot_step(time_step)


class DistanceSensor:
    def __init__(self, name: str):
        self.id = Robot.controller.wb_robot_get_device(str.encode(name))
        Robot.controller.wb_distance_sensor_get_value.restype = ctypes.c_double

    def enable(self, time_step: int):
        Robot.controller.wb_distance_sensor_enable(self.id, time_step)

    @property
    def value(self) -> float:
        return Robot.controller.wb_distance_sensor_get_value(self.id)


class Motor:
    def __init__(self, name: str):
        self.id = Robot.controller.wb_robot_get_device(str.encode(name))
        Robot.controller.wb_motor_get_target_position.restype = ctypes.c_double
        Robot.controller.wb_motor_get_velocity.restype = ctypes.c_double

    @property
    def position(self) -> float:
        return Robot.controller.wb_motor_get_target_position(self.id)

    @position.setter
    def position(self, p: float):
        Robot.controller.wb_motor_set_position(self.id, ctypes.c_double(p))

    @property
    def velocity(self) -> float:
        return Robot.controller.wb_motor_get_velocity(self.id)

    @velocity.setter
    def velocity(self, v: float):
        Robot.controller.wb_motor_set_velocity(self.id, ctypes.c_double(v))
