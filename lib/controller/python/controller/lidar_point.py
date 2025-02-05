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


class LidarPoint:
    def __init__(self, data: bytes, offset):
        t = struct.unpack_from('fffif', data, offset * 20)
        self.x = t[0]
        self.y = t[1]
        self.z = t[2]
        self.layer = t[3]
        self.time = t[4]
