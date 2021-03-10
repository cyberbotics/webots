#!/usr/bin/env python3

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

"""Convert Webots PROTO file from the RUB (x-Right, y-Up, z-Back) to FLU (x-Forward, y-Left, z-Up)."""

import sys
import numpy as np
import transforms3d
from webots_parser import WebotsParser


ROTATION_RUB_TO_FLU = transforms3d.axangles.axangle2mat(
    [-0.5773516025189619, 0.5773476025217157, 0.5773516025189619], -2.094405307179586)
ROTATION_KEYWORD = '  '*5 + 'rotation '
TRANSLATION_KEYWORD = '  '*5 + 'translation '


parser = WebotsParser()
parser.load(sys.argv[1])
exit

lines = None
with open(sys.argv[1], 'r') as f:
    lines = f.readlines()
    i = 0
    while i < len(lines):
        if lines[i].startswith(TRANSLATION_KEYWORD) and 'Transform' in lines[i-1]:
            translation_str = lines[i].replace(TRANSLATION_KEYWORD, '').strip()
            translation = [float(v) for v in translation_str.split(' ')]

            should_replace_rotation = False
            rotation_angle_axis = [1, 0, 0, 0]
            if ROTATION_KEYWORD in lines[i+1]:
                rotation_str = lines[i+1].replace(ROTATION_KEYWORD, '').strip()
                rotation_angle_axis = [float(v) for v in rotation_str.split(' ')]
                should_replace_rotation = True
            rotation = transforms3d.axangles.axangle2mat(rotation_angle_axis[:3], rotation_angle_axis[3])

            new_rotation = ROTATION_RUB_TO_FLU @ rotation
            new_rotation_axis, new_rotation_angle = transforms3d.axangles.mat2axangle(new_rotation)
            new_translation = ROTATION_RUB_TO_FLU @ np.array(translation)

            lines[i] = TRANSLATION_KEYWORD + ' '.join([f'{round(v, 3):.3}' for v in new_translation]) + '\n'
            new_rotation_str = ROTATION_KEYWORD + \
                ' '.join([f'{round(v, 4):.4}' for v in new_rotation_axis.tolist() + [new_rotation_angle]]) + '\n'
            if should_replace_rotation:
                lines[i+1] = new_rotation_str
            else:
                lines.insert(i, new_rotation_str)
            i += 1
        i += 1

with open(sys.argv[1], 'w') as fw:
    fw.writelines(lines)
