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

"""Test that the URDF exported by a robot describes it at any joint position.

The joints are moved away from their zero position, then the forward kinematics computed from the exported URDF must
give the joint anchors and the link orientations computed by Webots.
"""

import math
import os
import sys
import xml.etree.ElementTree as ET
from controller import Supervisor

TEST_NAME = os.path.splitext(os.path.basename(sys.argv[0]))[0] + '_' + sys.argv[1]
RESULTS_FILENAME = '../../../output.txt'
POSITION_TOLERANCE = 1e-4  # [m]
ROTATION_TOLERANCE = 1e-3  # [rad]

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())
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


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def add(u, v):
    return [u[i] + v[i] for i in range(3)]


def distance(u, v):
    return math.sqrt(sum((u[i] - v[i]) ** 2 for i in range(3)))


def rotation_from_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def rotation_from_axis_angle(axis, angle):
    norm = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / norm for a in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def rotation_difference(a, b):
    """Return the angle of the rotation between the orientations a and b."""
    trace = sum(a[k][i] * b[k][i] for i in range(3) for k in range(3))
    return math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))


def vector(string):
    return [float(v) for v in string.split()]


def field(node, name):
    f = node.getBaseNodeField(name) if node.isProto() else None
    return f if f is not None else node.getField(name)


def orientation(node):
    o = node.getOrientation()
    return [o[0:3], o[3:6], o[6:9]]


# find the joints of the robot, the URDF name of a joint is the name of one of its devices
joints = []
joints_by_name = {}


def visit(node):
    if node.getBaseTypeName() in ('HingeJoint', 'SliderJoint'):
        end_point = field(node, 'endPoint').getSFNode()
        joint = {'node': node, 'slider': node.getBaseTypeName() == 'SliderJoint', 'end_point': end_point,
                 'parameters': field(node, 'jointParameters').getSFNode(), 'motor': None}
        devices = field(node, 'device')
        for i in range(devices.getCount()):
            device = devices.getMFNode(i)
            name = field(device, 'name').getSFString()
            joints_by_name[name] = joint
            if device.getBaseTypeName() in ('RotationalMotor', 'LinearMotor'):
                joint['motor'] = robot.getDevice(name)
        joints.append(joint)
        if end_point is not None:
            visit(end_point)
        return
    children = field(node, 'children')
    if children is not None and children.getTypeName() == 'MFNode':
        for i in range(children.getCount()):
            visit(children.getMFNode(i))


notify(True)
visit(robot.getSelf())
if not joints:
    finish('No joint found.')

# move the joints away from their zero position
for joint in joints:
    motor = joint['motor']
    low, high = (motor.getMinPosition(), motor.getMaxPosition()) if motor else (0.0, 0.0)
    if low == high:
        low, high = (0.0, 0.02) if joint['slider'] else (-1.0, 1.0)
    if not joint['slider']:
        low, high = max(low, -2.0), min(high, 2.0)
    position = low + 0.3 * (high - low)
    joint['node'].setJointPosition(position)
    if motor:
        motor.setPosition(position)
for _ in range(3):
    robot.step(time_step)

# poses computed by Webots
for joint in joints:
    parameters = joint['parameters']
    joint['position'] = field(parameters, 'position').getSFFloat() if parameters else 0.0
    if not joint['slider']:
        parent = joint['node'].getParentNode()
        anchor = field(parameters, 'anchor').getSFVec3f() if parameters else [0.0, 0.0, 0.0]
        joint['anchor'] = add(parent.getPosition(), rotate(orientation(parent), anchor))

# forward kinematics of the URDF
urdf = ET.fromstring(robot.getUrdf())
children = {}
child_links = set()
for element in urdf.findall('joint'):
    children.setdefault(element.find('parent').get('link'), []).append(element)
    child_links.add(element.find('child').get('link'))
roots = [link.get('name') for link in urdf.findall('link') if link.get('name') not in child_links]
if len(roots) != 1:
    finish(f'The URDF file should have one root link, found {len(roots)}.')

errors = []
checked = set()


def check(link, position, rotation):
    for element in children.get(link, []):
        origin = element.find('origin')
        child_position = add(position, rotate(rotation, vector(origin.get('xyz', '0 0 0'))))
        child_rotation = multiply(rotation, rotation_from_rpy(*vector(origin.get('rpy', '0 0 0'))))
        if element.get('type') != 'fixed':
            name = element.get('name')
            joint = joints_by_name.get(name)
            if joint is None:
                errors.append(f'URDF joint "{name}" does not match a joint of the robot')
                continue
            checked.add(id(joint))
            axis = vector(element.find('axis').get('xyz'))
            end_point = joint['end_point']
            if joint['slider']:
                child_position = add(child_position, rotate(child_rotation, [a * joint['position'] for a in axis]))
                expected_position = end_point.getPosition()
            else:
                child_rotation = multiply(child_rotation, rotation_from_axis_angle(axis, joint['position']))
                expected_position = joint['anchor']
            position_error = distance(child_position, expected_position)
            rotation_error = rotation_difference(child_rotation, orientation(end_point))
            if position_error > POSITION_TOLERANCE or rotation_error > ROTATION_TOLERANCE:
                errors.append(f'joint "{name}": position error {1000 * position_error:.3f} mm, '
                              f'rotation error {math.degrees(rotation_error):.3f} deg')
        check(element.find('child').get('link'), child_position, child_rotation)


check(roots[0], robot.getSelf().getPosition(), orientation(robot.getSelf()))
if len(checked) != len(joints):
    errors.append(f'{len(joints) - len(checked)} joint(s) of the robot are not in the URDF file')
finish('; '.join(errors) if errors else None)
