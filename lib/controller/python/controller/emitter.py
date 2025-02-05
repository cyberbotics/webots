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

import struct
import sys
from .wb import wb
from .device import Device
import ctypes
from typing import Union, List


class Emitter(Device):
    wb.wb_emitter_get_range.restype = ctypes.c_double

    CHANNEL_BROADCAST = -1

    def __init__(self, name: Union[str, int]):
        super().__init__(name)

    def getBufferSize(self) -> int:
        return self.buffer_size

    @property
    def buffer_size(self) -> int:
        return wb.wb_emitter_get_buffer_size(self._tag)

    def send(self, message: Union[str, bytes, List[float]], length: int = None):
        if isinstance(message, bytes):
            if length is None:
                length = len(message)
            wb.wb_emitter_send(self._tag, message, length)
        elif isinstance(message, str):
            wb.wb_emitter_send(self._tag, str.encode(message), len(message))
        elif isinstance(message, list) or isinstance(message, tuple):
            length = len(message)
            if length == 0:
                print('Emitter.send(): empty list', file=sys.stderr)
                return
            if isinstance(message[0], float):
                data_type = 'd'
            elif isinstance(message[0], int):
                data_type = 'i'
            elif isinstance(message[0], bool):
                data_type = '?'
            else:
                print(f'Emitter.send(): unsupported data type list: {type(message)}', file=sys.stderr)
                return
            pack = struct.pack(f'{length}{data_type}', *message)
            wb.wb_emitter_send(self._tag, pack, len(pack))
        else:
            print(f'Emitter.send(): unsupported data type: {type(message)}', file=sys.stderr)

    def getChannel(self) -> int:
        return self.channel

    @property
    def channel(self) -> int:
        return wb.wb_emitter_get_channel(self._tag)

    def setChannel(self, channel: int):
        self.channel = channel

    @channel.setter
    def channel(self, c: int):
        wb.wb_emitter_set_channel(self._tag, c)

    def getRange(self) -> float:
        return self.range

    @property
    def range(self) -> float:
        return wb.wb_emitter_get_range(self._tag)

    def setRange(self, range: float):
        self.range = range

    @range.setter
    def range(self, range: float):
        wb.wb_emitter_set_range(self._tag, ctypes.c_double(range))
