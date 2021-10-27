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
import sys
from webots.wb import wb

wb.wb_robot_get_basic_time_step.restype = ctypes.c_double


class Robot:
    created = None

    def __init__(self):
        if Robot.created:
            print('Error: only one Robot instance can be created per controller process.', file=sys.stderr)
            return
        Robot.created = self
        wb.wb_robot_init()

    def step(self, time_step: int = None) -> int:
        if time_step is None:
            time_step = int(self.basicTimeStep)
        return wb.wb_robot_step(time_step)

    @property
    def basicTimeStep(self) -> float:
        return wb.wb_robot_get_basic_time_step()
