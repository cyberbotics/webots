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

import ctypes
from controller.wb import wb
from controller.sensor import Sensor


class Receiver(Sensor):
    def __init__(self, name: str, sampling_period: int = None):
        self._enable = wb.wb_receiver_enable
        self._get_sampling_period = wb.wb_receiver_get_sampling_period
        super().__init__(name, sampling_period)

    @property
    def queue_length(self) -> int:
        return wb.wb_receiver_get_queue_length(self._tag)

    wb.wb_receiver_get_data.restype = ctypes.c_char_p

    @property
    def data(self) -> bytes:
        return wb.wb_receiver_get_data(self._tag)

    @property
    def string(self) -> str:
        return self.data.decode()

    def next_packet(self):
        wb.wb_receiver_next_packet(self._tag)
