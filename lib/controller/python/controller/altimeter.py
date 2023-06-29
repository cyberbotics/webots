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

from .sensor import Sensor
from .wb import wb
import ctypes
import typing


class Altimeter(Sensor):
    wb.wb_altimeter_get_value.restype = ctypes.c_double

    def __init__(self, name: typing.Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_altimeter_enable
        self._get_sampling_period = wb.wb_altimeter_get_sampling_period
        super().__init__(name, sampling_period)

    def getValue(self) -> float:
        return self.value

    @property
    def value(self) -> float:
        return wb.wb_altimeter_get_value(self._tag)
