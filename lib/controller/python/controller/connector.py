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


class Connector(Device):
    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def disablePresence(self):
        wb.wb_connector_disable_presence(self._tag)

    def enablePresence(self, sampling_period):
        wb.wb_connector_enable_presence(self._tag, sampling_period)

    def getPresence(self) -> int:
        return self.presence

    def getPresenceSamplingPeriod(self) -> int:
        return self.presence_sampling_period

    def isLocked(self) -> bool:
        return self.locked

    def lock(self):
        wb.wb_connector_lock(self._tag)

    def unlock(self):
        wb.wb_connector_unlock(self._tag)

    @property
    def locked(self) -> bool:
        return wb.wb_connector_is_locked(self._tag) != 0

    @property
    def presence(self) -> int:
        return wb.wb_connector_get_presence(self._tag)

    @property
    def presence_sampling_period(self) -> int:
        return wb.wb_connector_get_presence_sampling_period(self._tag)
