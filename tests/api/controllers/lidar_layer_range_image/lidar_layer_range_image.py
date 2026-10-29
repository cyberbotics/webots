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

"""Test that Lidar.getLayerRangeImage and Lidar.getRangeImageArray return the range image of each layer."""

import os
import sys
from controller import Robot

TIME_STEP = 64
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


notify(True)

lidar = robot.getDevice('lidar')
lidar.enable(TIME_STEP)
robot.step(TIME_STEP)
robot.step(TIME_STEP)

width = lidar.getHorizontalResolution()
layers = lidar.getNumberOfLayers()
image = lidar.getRangeImage()
rows = [image[i * width:(i + 1) * width] for i in range(layers)]

# the test is only meaningful if the layers differ from each other
if all(row == rows[0] for row in rows):
    finish('The layers of the range image should differ from each other.')

array = lidar.getRangeImageArray()
if len(array) != layers:
    finish('getRangeImageArray should return %d layers, not %d.' % (layers, len(array)))

for i in range(layers):
    if lidar.getLayerRangeImage(i) != rows[i]:
        finish('getLayerRangeImage(%d) should be %s, not %s.' % (i, rows[i], lidar.getLayerRangeImage(i)))
    if array[i] != rows[i]:
        finish('getRangeImageArray()[%d] should be %s, not %s.' % (i, rows[i], array[i]))

# a layer that does not exist has no image
if lidar.getLayerRangeImage(layers) is not None:
    finish('getLayerRangeImage(%d) should be None, not %s.' % (layers, lidar.getLayerRangeImage(layers)))

finish(None)
