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


def find(function, elements, default=None):
    return next((x for x in elements if function(x)), default)


def get_translation(fields):
    translation_field = find(lambda x: x['name'] == 'translation', fields)
    if not translation_field:
        return [0, 0, 0]
    return [float(value) for value in translation_field['value']]


def get_rotation(fields):
    rotation_field = find(lambda x: x['name'] == 'rotation', fields)
    if not rotation_field:
        return [1, 0, 0, 0]
    return [float(value) for value in rotation_field['value']]


def set_rotation(fields, value):
    rotation_field = find(lambda x: x['name'] == 'rotation', fields)
    if not rotation_field:
        fields.append({'name': 'rotation',
                       'value': value,
                       'type': 'SFRotation'})
        return
    rotation_field['value'] = value


def set_translation(fields, value):
    translation_field = find(lambda x: x['name'] == 'translation', fields)
    if not translation_field:
        fields.append({'name': 'translation',
                       'value': value,
                       'type': 'SFTranslation'})
        return
    translation_field['value'] = value


def convert_pose(rotation_angle_axis, translation):
    # Convert
    rotation = transforms3d.axangles.axangle2mat(rotation_angle_axis[:3], rotation_angle_axis[3])
    new_rotation = ROTATION_RUB_TO_FLU @ rotation
    new_rotation_axis, new_rotation_angle = transforms3d.axangles.mat2axangle(new_rotation)
    new_translation = ROTATION_RUB_TO_FLU @ np.array(translation)

    # Convert to string array
    new_rotation_str = [f'{round(v, 4):.4}' for v in new_rotation_axis.tolist() + [new_rotation_angle]]
    new_translation_str = [f'{round(v, 3):.3}' for v in new_translation]

    return new_rotation_str, new_translation_str


def main():
    proto = WebotsParser()
    proto.load(sys.argv[1])

    robot_children = []
    for field in proto.content['root'][0]['fields']:
        if field['name'] == 'children':
            robot_children = field['value']

    for child in robot_children:
        if 'Hinge' not in child['name'] and child['name'] != 'Group' and child['name'] != 'Shape':
            translation = get_translation(child['fields'])
            rotation = get_rotation(child['fields'])

            new_rotation, new_translaton = convert_pose(rotation, translation)

            set_rotation(child['fields'], new_rotation)
            set_translation(child['fields'], new_translaton)

    proto.save(sys.argv[1])


if __name__ == '__main__':
    main()
