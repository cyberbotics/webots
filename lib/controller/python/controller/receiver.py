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
import struct
import sys
from .wb import wb
from .sensor import Sensor
from typing import Union, Tuple, List


class Receiver(Sensor):
    wb.wb_receiver_get_data.restype = ctypes.POINTER(ctypes.c_ubyte)
    wb.wb_receiver_get_signal_strength.restype = ctypes.c_double
    wb.wb_receiver_get_emitter_direction.restype = ctypes.POINTER(ctypes.c_double)

    def __init__(self, name: Union[str, int], sampling_period: int = None):
        self._enable = wb.wb_receiver_enable
        self._get_sampling_period = wb.wb_receiver_get_sampling_period
        super().__init__(name, sampling_period)

    def getBytes(self) -> bytes:
        return bytes(self.data[0:self.data_size])

    def getData(self) -> str:
        print('DEPRECATION: Receiver.getData is deprecated, please use Receiver.getString instead.', file=sys.stderr)
        return self.string

    def getDataSize(self) -> int:
        return self.data_size

    def getFloats(self) -> Tuple[float, ...]:
        return struct.unpack(f'{int(self.data_size / 8)}d', self.getBytes())

    def getInts(self) -> Tuple[int, ...]:
        return struct.unpack(f'{int(self.data_size / 4)}i', self.getBytes())

    def getBools(self) -> Tuple[bool, ...]:
        return struct.unpack(f'{self.data_size}?', self.getBytes())

    def getQueueLength(self) -> int:
        return self.queue_length

    def getString(self) -> str:
        return self.string

    def nextPacket(self):
        wb.wb_receiver_next_packet(self._tag)

    def getSignalStrength(self):
        return self.signal_strength

    def getEmitterDirection(self) -> List[float]:
        return self.emitter_direction

    def getChannel(self) -> int:
        return self.channel

    def setChannel(self, channel: int):
        self.channel = channel

    @property
    def queue_length(self) -> int:
        return wb.wb_receiver_get_queue_length(self._tag)

    @property
    def data(self) -> bytes:
        return wb.wb_receiver_get_data(self._tag)

    @property
    def data_size(self) -> int:
        return wb.wb_receiver_get_data_size(self._tag)

    @property
    def string(self) -> str:
        return ctypes.string_at(self.data, self.data_size).decode()

    @property
    def signal_strength(self) -> float:
        return wb.wb_receiver_get_signal_strength(self._tag)

    @property
    def emitter_direction(self) -> List[float]:
        return wb.wb_receiver_get_emitter_direction(self._tag)[:3]

    @property
    def channel(self) -> int:
        return wb.wb_receiver_get_channel(self._tag)

    @channel.setter
    def channel(self, channel: int):
        wb.wb_receiver_set_channel(self._tag, channel)
