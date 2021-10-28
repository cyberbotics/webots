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

import typing
from webots.wb import wb


class Sensor:
    def __init__(self, name: str, samplingPeriod: int = None):
        self._tag = wb.wb_robot_get_device(str.encode(name))
        self.samplingPeriod = int(wb.wb_robot_get_basic_time_step()) if samplingPeriod is None else samplingPeriod

    @property
    def samplingPeriod(self) -> int:
        return self._get_sampling_period(self._tag)

    @samplingPeriod.setter
    def samplingPeriod(self, p: typing.Union[int, None]):
        if p is None:
            p = 0
        self._enable(self._tag, p)
