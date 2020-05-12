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

import math
import numpy
import sys

from webots import WebotsModel


def translation(value):
    return [value[2], value[0], value[1]]


def axis_angle_to_quaternion(axis, theta):
    axis = numpy.array(axis) / numpy.linalg.norm(axis)
    return numpy.append([numpy.cos(theta/2)], numpy.sin(theta/2) * axis)


def quaternion_multiply(q1, q2):
    q3 = numpy.copy(q1)
    q3[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    q3[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]
    q3[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]
    q3[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
    return q3


def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = math.sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def quaternion_to_axis_angle(q):
    w, v = q[0], q[1:]
    theta = math.acos(w) * 2.0
    return normalize(v), theta


def rotation(value):
    q0 = axis_angle_to_quaternion([value[2], value[0], value[1]], value[3])
    q1 = axis_angle_to_quaternion([1, 0, 0], math.pi / 2)
    q2 = axis_angle_to_quaternion([0, 1, 0], math.pi / 2)
    qr = quaternion_multiply(q0, q1)
    qr = quaternion_multiply(qr, q2)
    (v, theta) = quaternion_to_axis_angle(qr)
    return [v[0], v[1], v[2], theta]


filename = sys.argv[1]
world = WebotsModel()
world.load(filename)
for node in world.content['root']:
    if node['name'] == 'WorldInfo':
        found = False
        g = [0.0, 0.0, -9.81]
        for field in node['fields']:
            if field['name'] == 'gravity':
                field['value'] = g
                found = True
                break
        if not found:
            gravity = {}
            gravity['name'] = 'gravity'
            gravity['type'] = 'SFVec3f'
            gravity['value'] = g
            node['fields'].append(gravity)
    else:
        for field in node['fields']:
            if field['name'] in ['translation', 'position', 'location', 'direction']:
                field['value'] = translation(field['value'])
            elif field['name'] in ['rotation', 'orientation']:
                field['value'] = rotation(field['value'])
world.save(filename[:-4] + '_enu.wbt')
