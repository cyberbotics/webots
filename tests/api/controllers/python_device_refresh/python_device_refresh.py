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

"""Test that Robot.getDevice and Robot.getDeviceByIndex find devices imported after the controller started."""

import os
import sys
from controller import Supervisor

TIME_STEP = 64
TEST_NAME = os.path.splitext(os.path.basename(sys.argv[0]))[0]
RESULTS_FILENAME = '../../../output.txt'

robot = Supervisor()
emitter = robot.getDevice('ts_emitter')


def notify(running):
    emitter.send(f'ts {int(running)} {os.getpid()}'.encode())


def finish(message):
    success = message is None
    with open(RESULTS_FILENAME, 'a') as f:
        f.write('OK: ' + TEST_NAME + '\n' if success else 'FAILURE with ' + TEST_NAME + ': ' + message + '\n')
    print('OK: ' + TEST_NAME if success else 'FAILURE with ' + TEST_NAME + ': ' + message)
    notify(False)
    sys.exit(0 if success else 1)


notify(True)

devices_at_start = robot.getNumberOfDevices()

children = robot.getSelf().getField('children')

# getDevice must find a device imported after startup on its own, before anything else refreshed the devices
children.importMFNodeFromString(-1, 'DistanceSensor { name "late_sensor" }')
robot.step(TIME_STEP)
if robot.getNumberOfDevices() != devices_at_start + 1:
    finish('The number of devices should be %d after the first import, not %d.' %
           (devices_at_start + 1, robot.getNumberOfDevices()))
sensor = robot.getDevice('late_sensor')
if sensor is None:
    finish('Robot.getDevice did not find the imported device "late_sensor".')
if type(sensor).__name__ != 'DistanceSensor':
    finish('Robot.getDevice returned a %s for "late_sensor", expected a DistanceSensor.' % type(sensor).__name__)

# likewise getDeviceByIndex, before any getDevice call has refreshed the devices
children.importMFNodeFromString(-1, 'Camera { name "late_camera" width 4 height 4 }')
robot.step(TIME_STEP)
if robot.getNumberOfDevices() != devices_at_start + 2:
    finish('The number of devices should be %d after the second import, not %d.' %
           (devices_at_start + 2, robot.getNumberOfDevices()))
try:
    camera = robot.getDeviceByIndex(devices_at_start + 1)
except KeyError:
    finish('Robot.getDeviceByIndex did not find the imported camera.')
if type(camera).__name__ != 'Camera':
    finish('Robot.getDeviceByIndex returned a %s for the imported camera, expected a Camera.' % type(camera).__name__)
if camera is not robot.getDevice('late_camera'):
    finish('Robot.getDeviceByIndex and Robot.getDevice returned different objects for "late_camera".')

sensor.enable(TIME_STEP)
robot.step(TIME_STEP)
sensor.getValue()

finish(None)
