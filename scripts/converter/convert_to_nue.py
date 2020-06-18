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

'''Convert world file from R2020a to R2020b keeping the NUE coordinate system.'''

import sys

from transforms3d import quaternions

from webots_parser import WebotsParser
from converted_protos import ConvertedProtos


def translation(value):
    return [value[2], value[0], value[1]]


def rotation_axis(value):
    return [value[2], value[0], value[1], value[3]]


def rotation(value):
    value = rotation_axis(value)
    q0 = quaternions.axangle2quat([value[0], value[1], value[2]], value[3])
    qr = quaternions.qmult(q0, quaternions.qinverse([0.5, 0.5, 0.5, 0.5]))
    v, theta = quaternions.quat2axangle(qr)
    return [v[0], v[1], v[2], theta]


filename = sys.argv[1]
world = WebotsParser()
world.load(filename)

for node in world.content['root']:
    if node['name'] == 'WorldInfo':
        found_gravity = False
        for field in node['fields']:
            if field['name'] == 'gravity':
                found_gravity = True
                field['value'] = -field['value'][1]
                field['type'] = 'SFFloat'
        if not found_gravity:
            node['fields'].append({'name': 'gravity', 'value': 9.81, 'type': 'SFFloat'})
        node['fields'].append({'name': 'coordinateSystem', 'value': 'NUE', 'type': 'SFString'})
    elif node['name'] in ConvertedProtos:
        for field in node['fields']:
            if field['name'] in ['translation']:
                field['value'] = translation(field['value'])
            elif field['name'] in ['rotation']:
                field['value'] = rotation_axis(field['value'], node['name'])

world.save(filename[:-4] + '_nue.wbt')
