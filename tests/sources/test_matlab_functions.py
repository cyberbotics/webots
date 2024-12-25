#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Test that all the required Matlab functions are defined."""
import unittest
import os
import re
import sys

# Add the parent directory to the path so we can import the command module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from command import Command  # noqa: E402

WEBOTS_HOME = os.path.normpath(os.environ['WEBOTS_HOME'])
sys.path.append(os.path.join(WEBOTS_HOME, 'src', 'controller', 'matlab'))
import mgenerate  # noqa: E402


class TestMatlabFunctions(unittest.TestCase):
    """Unit test for checking that all the required Matlab functions are defined."""

    def setUp(self):
        if sys.version_info[0] >= 3:
            mgenerate.UPDATE = False
            mgenerate.main()
            """Get all the required functions."""
            skippedLines = [
                'microphone', 'radio',  # Experimental Node Types
                'remote_control', 'wbr',  # Remote Control Plugin
                'robot',  # Many robot functions are used internally by the C API (e.x. wb_robot_mutex_*)
                'lookup_table_size',  # Matlab lets you get the size of an array, so these functions are not needed
                'wbu_string'  # String manipulation functions are not needed
            ]
            skippedFunctions = [
                # These functions are used internally by the Matlab API
                'wb_camera_recognition_get_object',
                'wb_lidar_get_point',
                'wb_mouse_get_state_pointer',
                'wb_radar_get_target',

                'wb_device_get_type',  # Deprecated since 8.0.0
                'wb_node_get_name',  # C API Only

                # The Matlab API exposes the image data as a multidimensional array, so these functions are not needed
                'wb_camera_image_get_red',
                'wb_camera_image_get_green',
                'wb_camera_image_get_blue',
                'wb_camera_image_get_gray',
                'wb_camera_image_get_grey',

                # Not Yet Implemented
                'wbu_system_tmpdir',
                'wbu_system_webots_instance_path',
            ]
            self.functions = []

            command = Command([
                # Search for function definitions
                'grep', '-Eho', r'\b\w+[ *]+\w+\(',
                # In the controller headers
                os.path.join(WEBOTS_HOME, 'include', 'controller', 'c', 'webots', '*.h'),
                os.path.join(WEBOTS_HOME, 'include', 'controller', 'c', 'webots', 'utils', '*.h')
                ])
            command.run()
            if command.returncode != 0:
                self.fail(f'Failed to generate function list: {command.output}')

            for line in command.output.splitlines():
                # Filter out the function name
                function = re.sub(r'\b\w+[ *]+(\w+)\(', r'\1', line.strip())
                if (function.startswith('wb') and not any(skippedLine in function for skippedLine in skippedLines) and
                        not function[3:].isupper()):
                    if function not in skippedFunctions and function not in self.functions:
                        self.functions.append(function)

    @unittest.skipIf(sys.version_info[0] < 3, "not supported by Python 2.7")
    def test_matlab_function_exists(self):
        """Test that the function file exists."""
        for function in self.functions:
            filename = os.path.join(WEBOTS_HOME, 'lib', 'controller', 'matlab', function + '.m')
            self.assertTrue(
                os.path.isfile(filename),
                msg='Missing "%s" file.' % filename
            )


if __name__ == '__main__':
    unittest.main()
