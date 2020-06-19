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

'''Convert world files from R2020a to R2020b with either NUE or ENU coordinate system.'''

import glob
import os
from convert_to_nue import convert_to_nue

WEBOTS_HOME = os.environ['WEBOTS_HOME']
nue_world_files = ['tests/*/worlds/*.wbt',
                   'projects/humans/c3d/worlds/*.wbt',
                   'projects/languages/ros/worlds/pioneer3at.wbt',
                   'projects/robots/*/*/worlds/*.wbt',
                   'projects/samples/*/worlds/*.wbt',
                   'projects/samples/*/*/worlds/*.wbt',
                   'projects/vehicles/worlds/*.wbt'
                   ]
for file in nue_world_files:
    for f in glob.glob(os.path.join(WEBOTS_HOME, file.replace('/', '\\'))):
        print(f)
        convert_to_nue(f)
