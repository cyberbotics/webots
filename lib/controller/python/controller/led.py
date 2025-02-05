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


class LED(Device):
    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def set(self, v: Union[bool, int]):
        self.value = v

    def get(self) -> int:
        return self.value

    @property
    def value(self) -> int:
        return wb.wb_led_get(self._tag)

    @value.setter
    def value(self, v: Union[bool, int]):
        if isinstance(v, bool):
            wb.wb_led_set(self._tag, 1 if v else 0)
        else:
            wb.wb_led_set(self._tag, v)
