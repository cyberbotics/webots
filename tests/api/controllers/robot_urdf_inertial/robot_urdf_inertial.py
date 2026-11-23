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

"""Test that the URDF export contains the mass, center of mass and inertia matrix of the solids having a Physics node."""

import math
import os
import sys
import xml.etree.ElementTree as ET
from controller import Robot

TEST_NAME = os.path.splitext(os.path.basename(sys.argv[0]))[0]
RESULTS_FILENAME = '../../../output.txt'

robot = Robot()
emitter = robot.getDevice('ts_emitter')


def notify(running):
    emitter.send(f'ts {int(running)} {os.getpid()}'.encode())


def finish(message):
    success = message is None
    with open(RESULTS_FILENAME, 'a') as f:
        f.write('OK: ' + TEST_NAME + '\n' if success else 'FAILURE with ' + TEST_NAME + ': ' + message + '\n')
    print('OK: ' + TEST_NAME if success else 'FAILURE with ' + TEST_NAME + ': ' + message)
    notify(False)
    robot.step(0)  # flush the final packet now, the supervisor waits for it
    sys.exit(0 if success else 1)


def box_inertia(mass, x, y, z):
    """Return the inertia matrix of a box about its center."""
    return [[mass * (y * y + z * z) / 12, 0, 0], [0, mass * (x * x + z * z) / 12, 0], [0, 0, mass * (x * x + y * y) / 12]]


def rotate_inertia(inertia, angle):
    """Return R * inertia * R^T, where R is a rotation of angle about the z axis."""
    c, s = math.cos(angle), math.sin(angle)
    r = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return [[sum(r[i][k] * inertia[k][m] * r[j][m] for k in range(3) for m in range(3)) for j in range(3)] for i in range(3)]


# expected mass, center of mass in the URDF link frame (located at the joint anchor) and inertia matrix about the center of mass
EXPECTED = {
    # density 1000 and a 0.2 x 0.1 x 0.05 box translated by (0.05, 0, 0.02)
    'base_link': (1.0, [0.05, 0, 0.02], box_inertia(1.0, 0.2, 0.1, 0.05)),
    # custom inertia matrix, the solid is 0.1 m above the anchor of its joint
    'link B': (2.0, [0.01, 0.02, 0.13], [[0.2, 0.01, 0.02], [0.01, 0.3, 0.03], [0.02, 0.03, 0.4]]),
    # box rotated by 0.5 rad, the solid is rotated by 0.3 rad about x and 0.1 m away from the anchor of its joint
    'link C': (3.0, [0, 0.1 * math.cos(0.3), -0.1 * math.sin(0.3)], rotate_inertia(box_inertia(3.0, 0.3, 0.1, 0.1), 0.5)),
    # center of mass given in the Physics node, the solid is 0.05 m away from the anchor of its joint
    'link D': (1.0, [0.15, 0, 0], box_inertia(1.0, 0.1, 0.2, 0.3)),
}

notify(True)
robot.step(int(robot.getBasicTimeStep()))
urdf = ET.fromstring(robot.getUrdf())
errors = []
for link in urdf.findall('link'):
    name = link.get('name')
    inertial = link.find('inertial')
    if name not in EXPECTED:
        if inertial is not None:
            errors.append(f'link "{name}" has no Physics node but has an inertial element')
        continue
    if inertial is None:
        errors.append(f'link "{name}" has no inertial element')
        continue
    mass, center_of_mass, inertia = EXPECTED.pop(name)
    exported_mass = float(inertial.find('mass').get('value'))
    exported_center_of_mass = [float(v) for v in inertial.find('origin').get('xyz').split()]
    i = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
    exported_inertia = [[i['ixx'], i['ixy'], i['ixz']], [i['ixy'], i['iyy'], i['iyz']], [i['ixz'], i['iyz'], i['izz']]]
    if abs(exported_mass - mass) > 1e-6:
        errors.append(f'link "{name}": mass {exported_mass} instead of {mass}')
    if max(abs(a - b) for a, b in zip(exported_center_of_mass, center_of_mass)) > 1e-6:
        errors.append(f'link "{name}": center of mass {exported_center_of_mass} instead of {center_of_mass}')
    if max(abs(exported_inertia[r][c] - inertia[r][c]) for r in range(3) for c in range(3)) > 1e-8:
        errors.append(f'link "{name}": inertia matrix {exported_inertia} instead of {inertia}')
for name in EXPECTED:
    errors.append(f'link "{name}" is missing')
finish('; '.join(errors) if errors else None)
