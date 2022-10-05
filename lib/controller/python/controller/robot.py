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
import sys
from controller.wb import wb
from controller.distance_sensor import DistanceSensor
from controller.constants import constant


class Robot:
    created = None
    wb.wb_device_get_name.restype = ctypes.c_char_p

    def __init__(self):
        if Robot.created:
            print('Error: only one Robot instance can be created per controller process.', file=sys.stderr)
            return
        Robot.created = self
        wb.wb_robot_init()
        self.NODE_DISTANCE_SENSOR = constant('NODE_DISTANCE_SENSOR')
        self.devices = {}
        n = wb.wb_robot_get_number_of_devices()
        for i in range(0, n):
            tag = wb.wb_robot_get_device_by_index(i)
            name = wb.wb_device_get_name(tag).decode()
            type = wb.wb_device_get_node_type(tag)
            if type == WB_NODE_DISTANCE_SENSOR:
                self.devices[name] = DistanceSensor(name)
            print(str(self.devices[name]) + ': ' + name)

    def getDistanceSensor(self, name: str) -> DistanceSensor:
        print('DEPRECATION: Robot.getDistanceSensor is deprecated, please use Robot.getDevice instead.', file=sys.stderr)
        return DistanceSensor(name)

    def getDevice(self, name: str):
        return 0

    def step(self, time_step: int = None) -> int:
        if time_step is None:
            time_step = int(self.basic_time_step)
        return wb.wb_robot_step(time_step)

    wb.wb_robot_get_basic_time_step.restype = ctypes.c_double

    @property
    def basic_time_step(self) -> float:
        return wb.wb_robot_get_basic_time_step()
