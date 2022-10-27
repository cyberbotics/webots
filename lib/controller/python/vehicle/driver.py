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

from controller import Supervisor
import ctypes
import os


class Driver(Supervisor):
    def __init__(self):
        super().__init__()
        ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'car.dll'))
        self.api = ctypes.cdll.LoadLibrary(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'driver.dll'))
        self.api.wbu_driver_get_cruising_speed.restype = ctypes.c_double
        self.api.wbu_driver_get_steering_angle.restype = ctypes.c_double
        self.api.wbu_driver_init()

    def getSteeringAngle(self) -> float:
        return self.steering_angle

    def setCruisingSpeed(self, cruisingSpeed: float):
        self.api.wbu_driver_set_cruising_speed(ctypes.c_double(cruisingSpeed))

    def setSteeringAngle(self, steeringAngle: float):
        self.steering_angle = steeringAngle

    cruising_speed = property(fset=setCruisingSpeed)

    @property
    def steering_angle(self) -> float:
        return self.api.wbu_driver_get_steering_angle()

    @steering_angle.setter
    def steering_angle(self, value: float):
        self.api.wbu_driver_set_steering_angle(ctypes.c_double(value))
