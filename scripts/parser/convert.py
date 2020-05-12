#!/usr/bin/env python3

# Copyright 1996-2020 Cyberbotics Ltd.
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

'''Convert world file from NUE to ENU coordinate system.'''

import sys
from webots import WebotsModel

filename = sys.argv[1]
world = WebotsModel()
world.load(filename)
for node in world.content['root']:
    if node['name'] == 'Viewpoint':
        print("coucou")
world.save("output.wrl")
