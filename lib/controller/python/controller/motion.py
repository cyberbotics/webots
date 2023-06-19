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

import ctypes
from .wb import wb


class Motion:
    wb.wbu_motion_new.restype = ctypes.c_void_p

    def __init__(self, filename: str):
        self._ref = ctypes.c_void_p(wb.wbu_motion_new(str.encode(filename)))

    def __del__(self):
        wb.wbu_motion_delete(self._ref)

    def isValid(self):
        return self._ref != ctypes.c_void_p(0)

    def play(self):
        wb.wbu_motion_play(self._ref)

    def stop(self):
        wb.wbu_motion_stop(self._ref)

    def setLoop(self, loop: bool):
        wb.wbu_motion_set_loop(self._ref, 1 if loop else 0)

    def setReverse(self, reverse: bool):
        wb.wbu_motion_set_reverse(self._ref, 1 if reverse else 0)

    def isOver(self) -> bool:
        return wb.wbu_motion_is_over(self._ref) != 0

    def getDuration(self) -> int:
        return self.duration

    def getTime(self) -> int:
        return self.time

    def setTime(self, time: int):
        self.time = time

    @property
    def time(self) -> int:
        return wb.wbu_motion_get_time(self._ref)

    @time.setter
    def time(self, time):
        wb.wbu_motion_set_time(self._ref, time)

    @property
    def duration(self) -> int:
        return wb.wbu_motion_get_duration(self._ref)
