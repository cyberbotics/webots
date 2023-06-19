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
from .device import Device
from typing import Union


class Sensor(Device):
    def __init__(self, name: Union[str, int], sampling_period: int = None):
        super().__init__(name)
        if isinstance(name, str):
            self.sampling_period = int(wb.wb_robot_get_basic_time_step()) if sampling_period is None else sampling_period

    def enable(self, p: int):
        self._enable(self._tag, p)

    def getSamplingPeriod(self) -> int:
        return self.sampling_period

    def disable(self):
        self._enable(self._tag, 0)

    @property
    def sampling_period(self) -> int:
        return self._get_sampling_period(self._tag)

    @sampling_period.setter
    def sampling_period(self, p: Union[int, None]):
        if p is None:
            p = 0
        self._enable(self._tag, p)
