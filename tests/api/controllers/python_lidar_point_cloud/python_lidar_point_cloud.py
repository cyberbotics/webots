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

"""Test the Python lidar point cloud getters, including unavailable data and buffer decoding."""

import math
import os
import struct
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


def point_values(point):
    return (point.x, point.y, point.z, point.layer, point.time)


def same_points(actual, expected):
    # Rays that do not hit an object can contain non-finite coordinates.
    return len(actual) == len(expected) and all(
        all(a == b or (math.isnan(a) and math.isnan(b)) for a, b in zip(point, other))
        for point, other in zip(actual, expected)
    )


def test_point_cloud():
    lidar = robot.getDevice('lidar')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()

    # The C API has not mapped the image data before the first step.
    if lidar.getRangeImage() is not None:
        finish('getRangeImage should return None before the first step.')
    if lidar.getRangeImageArray() is not None:
        finish('getRangeImageArray should return None before the first step.')
    if lidar.getLayerRangeImage(0) is not None:
        finish('getLayerRangeImage should return None before the first step.')
    for data_type in ('list', 'buffer'):
        if lidar.getPointCloud(data_type) is not None:
            finish('getPointCloud should return None before the first step.')
        if lidar.getLayerPointCloud(0, data_type) is not None:
            finish('getLayerPointCloud should return None before the first step.')

    if robot.step(TIME_STEP) == -1:
        raise RuntimeError('The simulation ended before the first sample.')
    count = lidar.getNumberOfPoints()
    width = lidar.getHorizontalResolution()
    layers = lidar.getNumberOfLayers()
    if count != width * layers:
        finish('The number of points should include every layer.')
    if len(lidar.getRangeImage()) != count:
        finish('The range image should contain one value per point.')

    points = lidar.getPointCloud()
    buffer = lidar.getPointCloud(data_type='buffer')
    if not isinstance(points, list) or len(points) != count:
        finish('The list should contain every point.')
    if not isinstance(buffer, bytes) or len(buffer) != 20 * count:
        finish('The buffer should contain 20 bytes per point.')
    values = [point_values(point) for point in points]
    if not same_points(values, list(struct.iter_unpack('fffif', buffer))):
        finish('List and buffer points should match.')
    if not any(all(math.isfinite(v) for v in point[:3]) and any(v != 0 for v in point[:3]) for point in values):
        finish('The test scene should produce a sampled, finite point.')

    for layer in range(layers):
        start = layer * width
        end = start + width
        layer_points = lidar.getLayerPointCloud(layer)
        layer_buffer = lidar.getLayerPointCloud(layer, data_type='buffer')
        if not same_points([point_values(point) for point in layer_points], values[start:end]):
            finish('Layer %d should match the corresponding full-cloud slice.' % layer)
        if layer_buffer != buffer[20 * start:20 * end]:
            finish('Layer %d buffers should match.' % layer)

    for data_type in ('list', 'buffer'):
        for layer in (-1, layers):
            if lidar.getLayerPointCloud(layer, data_type) is not None:
                finish('An invalid layer should return None.')
    if lidar.getPointCloud(data_type='invalid') is not None:
        finish('An invalid data_type should return None.')
    if lidar.getLayerPointCloud(0, data_type='invalid') is not None:
        finish('An invalid layer data_type should return None.')

    lidar.disablePointCloud()
    for data_type in ('list', 'buffer'):
        if lidar.getPointCloud(data_type) is not None:
            finish('A disabled point cloud should return None.')
        if lidar.getLayerPointCloud(0, data_type) is not None:
            finish('A disabled layer point cloud should return None.')
    lidar.enablePointCloud()
    lidar.disable()
    for data_type in ('list', 'buffer'):
        if lidar.getPointCloud(data_type) is not None:
            finish('A disabled lidar should have no point cloud.')
        if lidar.getLayerPointCloud(0, data_type) is not None:
            finish('A disabled lidar should have no layer point cloud.')


notify(True)
try:
    test_point_cloud()
except Exception as error:
    finish(str(error))
finish(None)
