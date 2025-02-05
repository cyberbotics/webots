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
from .device import Device
from typing import Union


class VacuumGripper(Device):
    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def disablePresence(self):
        wb.wb_vacuum_gripper_disable_presence(self._tag)

    def enablePresence(self, sampling_period):
        wb.wb_vacuum_gripper_enable_presence(self._tag, sampling_period)

    def getPresence(self) -> bool:
        return self.presence

    def getPresenceSamplingPeriod(self) -> int:
        return self.presence_sampling_period

    def isOn(self) -> bool:
        return self.on

    def turnOn(self):
        wb.wb_vacuum_gripper_turn_on(self._tag)

    def turnOff(self):
        wb.wb_vacuum_gripper_turn_off(self._tag)

    @property
    def on(self) -> bool:
        return wb.wb_vacuum_gripper_is_on(self._tag) != 0

    @property
    def presence(self) -> bool:
        return wb.wb_vacuum_gripper_get_presence(self._tag) != 0

    @property
    def presence_sampling_period(self) -> int:
        return wb.wb_vacuum_gripper_get_presence_sampling_period(self._tag)
