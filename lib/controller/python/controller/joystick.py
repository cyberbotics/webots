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

from .wb import wb
import ctypes
from typing import Union


class Joystick():
    wb.wb_joystick_get_model.restype = ctypes.c_char_p

    def __init__(self, sampling_period: int = None):
        if sampling_period != 0:
            self.sampling_period = int(wb.wb_robot_get_basic_time_step()) if sampling_period is None else sampling_period

    def enable(self, p: int):
        wb.wb_joystick_enable(p)

    def disable(self):
        wb.wb_joystick_disable()

    def getSamplingPeriod(self) -> int:
        return self.sampling_period

    def isConnected(self) -> bool:
        return self.is_connected

    def getNumberOfAxes(self) -> int:
        return self.number_of_axes

    def getAxisValue(self, axis: int) -> float:
        return wb.wb_joystick_get_axis_value(axis)

    def getNumberOfPovs(self) -> int:
        return self.number_of_povs

    def getPovValue(self, pov: int) -> int:
        return wb.wb_joystick_get_pov_value(pov)

    def getPressedButton(self) -> int:
        return self.pressed_button

    def setConstantForce(self, level: int):
        wb.wb_joystick_set_constant_force(level)

    def setConstantForceDuration(self, duration: float):
        wb.wb_joystick_set_constant_force_duration(ctypes.c_double(duration))

    def setAutoCenteringGain(self, gain: float):
        wb.wb_joystick_set_auto_centering_gain(ctypes.c_double(gain))

    def setResistanceGain(self, gain: float):
        wb.wb_joystick_set_resistance_gain(ctypes.c_double(gain))

    def setForceAxis(self, axis: int):
        wb.wb_joystick_set_force_axis(axis)

    @property
    def sampling_period(self) -> int:
        return wb.wb_joystick_get_sampling_period()

    @sampling_period.setter
    def sampling_period(self, p: Union[int, None]):
        if p is None:
            p = 0
        wb.wb_joystick_enable(p)

    @property
    def is_connected(self) -> bool:
        return wb.wb_joystick_is_connected() != 0

    @property
    def model(self) -> str:
        return wb.wb_joystick_get_model().decode()

    @property
    def number_of_axes(self) -> int:
        return wb.wb_joystick_get_number_of_axes()

    @property
    def number_of_povs(self) -> int:
        return wb.wb_joystick_get_number_of_povs()

    @property
    def pressed_button(self) -> int:
        return wb.wb_joystick_get_pressed_button()
