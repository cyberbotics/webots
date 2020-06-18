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
from converted_protos import converted_protos


def rotation(value, r):
    q0 = quaternions.axangle2quat([float(value[0]), float(value[1]), float(value[2])], float(value[3]))
    q1 = quaternions.axangle2quat([r[0], r[1], r[2]], r[3])
    qr = quaternions.qmult(q0, q1)
    v, theta = quaternions.quat2axangle(qr)
    return [WebotsParser.str(v[0]), WebotsParser.str(v[1]), WebotsParser.str(v[2]), WebotsParser.str(theta)]


filename = sys.argv[1]
world = WebotsParser()
world.load(filename)

for node in world.content['root']:
    if node['name'] == 'WorldInfo':
        for field in node['fields']:
            if field['name'] == 'gravity':
                field['value'] = -field['value'][1]
                field['type'] = 'SFFloat'
        node['fields'].append({'name': 'coordinateSystem', 'value': 'NUE', 'type': 'SFString'})
    elif node['name'] in converted_protos:
        print('Rotating', node['name'])
        rotation_found = False
        for field in node['fields']:
            if field['name'] in ['rotation']:
                rotation_found = True
                field['value'] = rotation(field['value'], converted_protos[node['name']])
        if not rotation_found:
            node['fields'].append({'name': 'rotation',
                                   'value': rotation(['0', '1', '0', '0'], converted_protos[node['name']]),
                                   'type': 'SFRotation'})

world.save(filename[:-4] + '_nue.wbt')
