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
    assert lidar.getRangeImage() is None, 'getRangeImage should return None before the first step.'
    for data_type in ('list', 'buffer'):
        assert lidar.getPointCloud(data_type) is None, 'getPointCloud should return None before the first step.'
        assert lidar.getLayerPointCloud(0, data_type) is None, \
            'getLayerPointCloud should return None before the first step.'

    if robot.step(TIME_STEP) == -1:
        raise RuntimeError('The simulation ended before the first sample.')
    count = lidar.getNumberOfPoints()
    width = lidar.getHorizontalResolution()
    layers = lidar.getNumberOfLayers()
    assert count == width * layers, 'The number of points should include every layer.'
    assert len(lidar.getRangeImage()) == count, 'The range image should contain one value per point.'

    points = lidar.getPointCloud()
    buffer = lidar.getPointCloud(data_type='buffer')
    assert isinstance(points, list) and len(points) == count, 'The list should contain every point.'
    assert isinstance(buffer, bytes) and len(buffer) == 20 * count, 'The buffer should contain 20 bytes per point.'
    values = [point_values(point) for point in points]
    assert same_points(values, list(struct.iter_unpack('fffif', buffer))), 'List and buffer points should match.'
    assert any(all(math.isfinite(v) for v in point[:3]) and any(v != 0 for v in point[:3]) for point in values), \
        'The test scene should produce a sampled, finite point.'

    for layer in range(layers):
        start = layer * width
        end = start + width
        layer_points = lidar.getLayerPointCloud(layer)
        layer_buffer = lidar.getLayerPointCloud(layer, data_type='buffer')
        assert same_points([point_values(point) for point in layer_points], values[start:end]), \
            'Layer %d should match the corresponding full-cloud slice.' % layer
        assert layer_buffer == buffer[20 * start:20 * end], 'Layer %d buffers should match.' % layer

    for data_type in ('list', 'buffer'):
        for layer in (-1, layers):
            assert lidar.getLayerPointCloud(layer, data_type) is None, 'An invalid layer should return None.'
    assert lidar.getPointCloud(data_type='invalid') is None, 'An invalid data_type should return None.'
    assert lidar.getLayerPointCloud(0, data_type='invalid') is None, 'An invalid layer data_type should return None.'

    lidar.disablePointCloud()
    for data_type in ('list', 'buffer'):
        assert lidar.getPointCloud(data_type) is None, 'A disabled point cloud should return None.'
        assert lidar.getLayerPointCloud(0, data_type) is None, 'A disabled layer point cloud should return None.'
    lidar.enablePointCloud()
    lidar.disable()
    for data_type in ('list', 'buffer'):
        assert lidar.getPointCloud(data_type) is None, 'A disabled lidar should have no point cloud.'
        assert lidar.getLayerPointCloud(0, data_type) is None, 'A disabled lidar should have no layer point cloud.'


notify(True)
try:
    test_point_cloud()
except Exception as error:
    finish(str(error))
finish(None)
