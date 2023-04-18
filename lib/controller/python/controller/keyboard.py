# Copyright 1996-2023 Cyberbotics Ltd.
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
from .constants import constant
from typing import Union


class Keyboard():
    END = constant('KEYBOARD_END')
    HOME = constant('KEYBOARD_HOME')
    LEFT = constant('KEYBOARD_LEFT')
    UP = constant('KEYBOARD_UP')
    RIGHT = constant('KEYBOARD_RIGHT')
    DOWN = constant('KEYBOARD_DOWN')
    PAGEUP = constant('KEYBOARD_PAGEUP')
    PAGEDOWN = constant('KEYBOARD_PAGEDOWN')
    NUMPAD_HOME = constant('KEYBOARD_NUMPAD_HOME')
    NUMPAD_LEFT = constant('KEYBOARD_NUMPAD_LEFT')
    NUMPAD_UP = constant('KEYBOARD_NUMPAD_UP')
    NUMPAD_RIGHT = constant('KEYBOARD_NUMPAD_RIGHT')
    NUMPAD_DOWN = constant('KEYBOARD_NUMPAD_DOWN')
    NUMPAD_END = constant('KEYBOARD_NUMPAD_END')
    KEY = constant('KEYBOARD_KEY')
    SHIFT = constant('KEYBOARD_SHIFT')
    CONTROL = constant('KEYBOARD_CONTROL')
    ALT = constant('KEYBOARD_ALT')

    def __init__(self, sampling_period: int = None):
        if sampling_period != 0:
            self.sampling_period = int(wb.wb_robot_get_basic_time_step()) if sampling_period is None else sampling_period

    def enable(self, p: int):
        wb.wb_keyboard_enable(p)

    def disable(self):
        wb.wb_keyboard_disable()

    def getSamplingPeriod(self) -> int:
        return self.sampling_period

    @property
    def sampling_period(self) -> int:
        return wb.wb_keyboard_get_sampling_period()

    @sampling_period.setter
    def sampling_period(self, p: Union[int, None]):
        if p is None:
            p = 0
        wb.wb_keyboard_enable(p)

    def getKeyCode(self) -> int:
        return wb.wb_keyboard_get_key()

    def getKey(self) -> int:
        k = wb.wb_keyboard_get_key()
        return k

    def get_key(self) -> str:
        k = wb.wb_keyboard_get_key()
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
